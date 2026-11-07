/*
 * ESP-01 (ESP8266) — дублёр управления вентилятором охлаждения автомобиля
 * через ELM327 по Wi-Fi (TCP). Неблокирующая реализация.
 *
 * Логика работы:
 *  - Пока валидная температура ОЖ читается — работает собственный гистерезис
 *    (вкл при T >= TEMP_ON, выкл при T <= TEMP_OFF).
 *  - Если валидных данных нет дольше RELEASE_CONTROL_TIMEOUT_MS (30 с) —
 *    система СНИМАЕТ свой управляющий сигнал (реле OFF) и отдаёт управление
 *    штатному ЭБУ. В чужое управление не вмешиваемся.
 *  - Как только данные вернулись — снова берём управление на себя.
 *  - При серии неудач ELM (ELM_MAX_FAILS подряд) — сначала мягкий сброс
 *    Wi-Fi. Если и это не помогает — полный ESP.restart().
 *  - При таймауте Wi-Fi — безопасный рестарт. Скан сетей делается один раз
 *    при старте, далее периодически печатается WiFi.status().
 *  - Отдельный детектор «мёртвого TCP»: если 0105 не отвечает N раз подряд
 *    (таймауты) или приходит N невалидных ответов подряд — TCP-сокет
 *    принудительно закрывается и открывается заново.
 *
 * Проводка реле: нормально разомкнутое реле включено параллельно штатному
 * управлению ЭБУ, поэтому «relay OFF» = мы не вмешиваемся в работу ЭБУ.
 * Рекомендуется внешний pull-down на затвор/базу транзистора реле, т.к.
 * GPIO2 до pinMode() находится в неопределённом состоянии.
 *
 * Возможности:
 *  - Неблокирующие FSM для Wi-Fi, TCP, инициализации ELM327 и опроса.
 *  - Автоматический перезапуск после N неудачных подключений к ELM.
 *  - Отладочный эмулятор температуры (без железа).
 *  - Отладочный режим "только скан Wi-Fi" (WIFI_SCAN_ONLY).
 */

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// --- Печать в Serial через operator<< (по мотивам cout) ---
// Serial << "текст" << значение << endl;  (endl == "\r\n", как у println)
template <typename T>
inline Print& operator<<(Print& out, const T& value) { out.print(value); return out; }

#define endl "\r\n"

// =============================================================
// РЕЖИМЫ ОТЛАДКИ (раскомментируйте нужный):
//
//  DEBUG_TEMP_EMULATOR — эмулятор температуры, Wi-Fi и ELM не задействованы.
//  WIFI_SCAN_ONLY      — только сканирование Wi-Fi сетей и печать списка,
//                        подключения нет. Удобно для поиска SSID адаптера.
//
//  ОБЫЧНЫЙ (рабочий) режим — когда оба макроса закомментированы.
// =============================================================
// #define DEBUG_TEMP_EMULATOR
// #define WIFI_SCAN_ONLY

// ======================= НАСТРОЙКИ =======================

// Wi-Fi точка доступа ELM327 (проверьте SSID и пароль вашего адаптера!)
const char* WIFI_SSID = "WIFI_OBDII";
const char* WIFI_PASS = "";

// TCP-адрес ELM327
IPAddress ELM_IP(192, 168, 0, 10);
const uint16_t ELM_PORT = 35000;

// Пин реле
const uint8_t RELAY_PIN = 2;      // GPIO2
const uint8_t RELAY_ON  = HIGH;
const uint8_t RELAY_OFF = LOW;

// Гистерезис по температуре ОЖ
const float TEMP_ON  = 95.0;
const float TEMP_OFF = 90.0;

// Допустимый диапазон температуры ОЖ (отбраковка мусорных ответов)
const float TEMP_MIN = -40.0;
const float TEMP_MAX = 150.0;

// Таймауты
const unsigned long WIFI_TIMEOUT_MS             = 30000;
const unsigned long ELM_PID_TIMEOUT_MS          = 1500;
const unsigned long POLL_INTERVAL_MS            = 1000;
const unsigned long RECONNECT_DELAY_MS          = 2000;
const unsigned long RELEASE_CONTROL_TIMEOUT_MS  = 30000;   // отдать управление ЭБУ
const unsigned long WIFI_SCAN_INTERVAL_MS       = 3000;    // период скана (WIFI_SCAN_ONLY)

const unsigned int MAX_RESP_LEN = 256;   // защита от неограниченного роста буфера ответа

const int ELM_MAX_FAILS = 5;      // подряд неудач -> мягкий сброс Wi-Fi
const int SOFT_RESETS_MAX = 2;    // мягких сбросов подряд без ELM_READY -> полный рестарт
const int POLL_FAILS_TO_RECONNECT = 4;   // 4 * 1.5s = ~6s без ответа -> force TCP reconnect
const int POLL_BAD_TO_RECONNECT   = 6;   // 6 невалидных ответов подряд -> force TCP reconnect

// ======================= СОСТОЯНИЯ =======================

enum WifiFSM { WIFI_DISCONNECTED, WIFI_CONNECTING, WIFI_CONNECTED };
WifiFSM wifiState = WIFI_DISCONNECTED;
unsigned long wifiStartMs = 0;
unsigned long wifiLastStatusMs = 0;
bool wifiScanDone = false;        // диагностический скан выполняется один раз

enum ElmFSM {
  ELM_DISCONNECTED,
  ELM_TCP_BACKOFF,
  ELM_INIT_SEND,
  ELM_INIT_WAIT,
  ELM_READY
};
ElmFSM elmState = ELM_DISCONNECTED;
unsigned long elmBackoffStartMs = 0;
int elmInitStep = 0;
int elmFailCount = 0;
int softResetCount = 0;           // мягких сбросов Wi-Fi подряд без выхода в ELM_READY

struct ElmInitCmd {
  const char*   cmd;
  unsigned long timeout;
};
const ElmInitCmd elmInitCmds[] = {
  { "ATZ",   15000 },
  { "ATE0",  5000 },
  { "ATL0",  2000 },
  { "ATH0",  2000 },
  { "ATSP0", 5000 },
};
const int ELM_INIT_COUNT = sizeof(elmInitCmds) / sizeof(elmInitCmds[0]);

String elmRespBuf;
unsigned long elmWaitStartMs = 0;
unsigned long elmCurrentTimeout = 0;

enum PollFSM { POLL_IDLE, POLL_WAIT };
PollFSM pollState = POLL_IDLE;
unsigned long lastPollMs = 0;
unsigned long pollSentMs = 0;
String pollRespBuf;
int pollFailStreak = 0;                   // счётчик подряд неудачных опросов (таймауты)
int pollBadStreak = 0;                    // счётчик подряд невалидных ответов

// Управление реле
bool relayState = false;

// Флаг «управление отдано ЭБУ»
bool controlReleased = false;
bool haveValidTemp = false;       // было ли хоть одно валидное чтение температуры
unsigned long lastValidTempMs = 0;

WiFiClient elmClient;

// ======================= ОБЩЕЕ =======================

// --- УПРАВЛЕНИЕ РЕЛЕ ---

void setRelay(bool on) {
  relayState = on;
  digitalWrite(RELAY_PIN, on ? RELAY_ON : RELAY_OFF);
  Serial << "Relay: " << (on ? "ON" : "OFF") << endl;
}

// Управление реле по гистерезису. Пока управление отдано ЭБУ — не вмешиваемся.
void controlRelay(float temp) {
  if (controlReleased) return;

  if (!relayState && temp >= TEMP_ON) {
    Serial << "Temp >= " << TEMP_ON << " -> fan ON" << endl;
    setRelay(true);
  } else if (relayState && temp <= TEMP_OFF) {
    Serial << "Temp <= " << TEMP_OFF << " -> fan OFF" << endl;
    setRelay(false);
  }
}

// Возврат к собственному управлению после появления валидной температуры.
void resumeOwnControl() {
  if (!controlReleased) return;
  Serial << "Valid temp restored -> resume own control" << endl;
  controlReleased = false;
  // Реле остаётся OFF. Если T уже >= TEMP_ON — controlRelay включит.
  // ВАЖНО: сброс в OFF безопасен только для нормально разомкнутого реле,
  // включённого ПАРАЛЛЕЛЬНО штатному управлению ЭБУ («OFF» = не вмешиваемся).
  // Для НЗ-реле последовательно с ЭБУ такая логика недопустима.
}

// --- РЕСТАРТ И ПЕРЕПОДКЛЮЧЕНИЕ TCP ---

// Безопасный рестарт: снять реле и закрыть TCP, потом перезагрузка.
void safeRestart(const char* reason) {
  Serial << "Restart: " << reason << endl;
  setRelay(false);
  elmClient.stop();   // безусловно: отпускаем TCP-слот на адаптере
  delay(150);         // дать реле гарантированно отпустить (дребезг контактов)
  ESP.restart();
}

// Принудительный перезапуск TCP-соединения (закрыть и подключиться заново).
void forceTcpReconnect(const char* reason) {
  Serial << reason << endl;
  elmClient.stop();
  elmState = ELM_TCP_BACKOFF;
  elmBackoffStartMs = millis();
  elmInitStep = 0;      // следующий connect начнёт инициализацию с ATZ
  elmFailCount++;
}

// --- ТАЙМЕРЫ (имя = что проверяем, не как) ---

// Истёк таймаут подключения к Wi-Fi.
bool wifiConnectTimedOut()   { return millis() - wifiStartMs > WIFI_TIMEOUT_MS; }
// Пора напечатать статус Wi-Fi во время подключения.
bool wifiStatusLogDue()      { return millis() - wifiLastStatusMs > 5000; }
// ELM не ответил на AT-команду за отведённое для неё время.
bool elmInitTimedOut()       { return millis() - elmWaitStartMs > elmCurrentTimeout; }
// Не пришёл ответ на 0105 за отведённое время.
bool pollResponseTimedOut()  { return millis() - pollSentMs > ELM_PID_TIMEOUT_MS; }
// Выдержана пауза перед повторным подключением к ELM.
bool elmBackoffElapsed()     { return millis() - elmBackoffStartMs > RECONNECT_DELAY_MS; }
// Пора отправить очередной запрос 0105.
bool pollDue()               { return millis() - lastPollMs >= POLL_INTERVAL_MS; }
// Валидной температуры нет слишком долго -> пора отдать управление ЭБУ.
bool noValidTempForTooLong() { return millis() - lastValidTempMs > RELEASE_CONTROL_TIMEOUT_MS; }

// --- СЧЁТЧИКИ НЕУДАЧ ---

// Сброс счётчиков неудачных опросов (таймауты и невалидные ответы).
void resetPollCounters() {
  pollFailStreak = 0;
  pollBadStreak  = 0;
}

// Зафиксировать невалидный ответ (увеличить счётчик).
void noteBadResponse() { pollBadStreak++; }

// Невалидных ответов подряд накопилось слишком много.
bool tooManyBadResponses() { return pollBadStreak >= POLL_BAD_TO_RECONNECT; }

// Зафиксировать таймаут опроса (увеличить счётчик).
void notePollTimeout() { pollFailStreak++; }

// Таймаутов опроса подряд накопилось слишком много («мёртвый TCP»).
bool tooManyPollTimeouts() { return pollFailStreak >= POLL_FAILS_TO_RECONNECT; }

// Слишком много неудач ELM подряд -> пора на мягкий сброс Wi-Fi.
bool tooManyElmFailures() { return elmFailCount >= ELM_MAX_FAILS; }

// --- ЖУРНАЛ WI-FI ---

// Человекочитаемое имя статуса Wi-Fi.
const char* wifiStatusName(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID";        // сеть с таким SSID не найдена
    case WL_SCAN_COMPLETED:  return "SCAN_DONE";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED"; // обычно неверный пароль
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_WRONG_PASSWORD:  return "WRONG_PASSWORD";
    case WL_DISCONNECTED:    return "DISCONNECTED";   // идёт попытка подключения
    default:                 return "UNKNOWN";
  }
}

// Печать "<prefix><код> (<имя>)" текущего статуса Wi-Fi.
void logWifiStatus(const char* prefix) {
  wl_status_t st = WiFi.status();
  Serial << prefix << (int)st << " (" << wifiStatusName(st) << ")" << endl;
}

// --- ПЕЧАТЬ СПИСКА СЕТЕЙ ---

// Печать списка видимых Wi-Fi сетей.
void printScanResult() {
  int n = WiFi.scanNetworks();
  Serial << "Scan: found " << n << " AP" << endl;
  for (int i = 0; i < n; i++) {
    Serial << "  [" << i << "] " << WiFi.SSID(i)
           << "  RSSI=" << WiFi.RSSI(i)
           << "  ch=" << WiFi.channel(i)
           << (WiFi.encryptionType(i) == ENC_TYPE_NONE ? "  open" : "  enc")
           << endl;
  }
}

// --- ПАРСЕР ОТВЕТА 0105 ---

// Значение hex-символа или -1, если это не hex-цифра.
int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Собрать байт из двух hex-символов. false — если хотя бы один не hex.
bool hexByte(char hi, char lo, long &v) {
  int h = hexVal(hi);
  int l = hexVal(lo);
  if (h < 0 || l < 0) return false;
  v = h * 16 + l;
  return true;
}

// Байт данных PID 0105 -> температура ОЖ в °C (по стандарту: значение - 40).
float pidByteToTemp(long v) {
  return (float)((int)v - 40);
}

// Парсинг ответа на 0105. Возвращает true и заполняет tempC при успехе.
bool parsePID0105(const String &resp, float &tempC) {
  // Нормализуем разделители
  String s = resp;
  s.replace('\r', ' ');
  s.replace('\n', ' ');
  s.replace('>', ' ');
  s.replace('\t', ' ');
  s = " " + s + " ";

  // Основной шаблон " 41 05 XX " (длина шаблона без завершающего '\0')
  static const char PATTERN[] = " 41 05 ";
  const int PATTERN_LEN = sizeof(PATTERN) - 1;
  int idx = s.indexOf(PATTERN);
  if (idx >= 0) {
    int vs = idx + PATTERN_LEN;
    while (vs < s.length() && s[vs] == ' ') vs++;
    if (vs + 2 <= s.length()) {
      long v = 0;
      if (hexByte(s[vs], s[vs + 1], v)) {
        tempC = pidByteToTemp(v);
        return true;
      }
    }
  }

  // Резерв: убираем пробелы и ищем "4105"
  static const char NOSPACE[] = "4105";
  const int NOSPACE_LEN = sizeof(NOSPACE) - 1;
  String c = resp;
  c.replace(" ", "");
  c.replace("\r", "");
  c.replace("\n", "");
  c.replace(">", "");
  int idx2 = c.indexOf(NOSPACE);
  if (idx2 >= 0 && idx2 + NOSPACE_LEN + 2 <= c.length()) {
    long v = 0;
    if (hexByte(c[idx2 + NOSPACE_LEN], c[idx2 + NOSPACE_LEN + 1], v)) {
      tempC = pidByteToTemp(v);
      return true;
    }
  }
  return false;
}

// Значение в допустимом диапазоне датчика ОЖ.
bool isValidCoolantTemp(float t) {
  return t >= TEMP_MIN && t <= TEMP_MAX;
}

// Считать валидную температуру ОЖ из ответа ELM: разбор + проверка диапазона.
bool readCoolantTemp(const String &resp, float &t) {
  return parsePID0105(resp, t) && isValidCoolantTemp(t);
}

// ======================= АВТОМАТ WI-FI =======================

void wifiTick() {
  switch (wifiState) {
    case WIFI_DISCONNECTED:
      Serial << "Wi-Fi connecting to " << WIFI_SSID << endl;

      // Диагностика: скан сетей делаем только один раз (скан блокирующий,
      // при флапании сети он бы тормозил основной цикл).
      if (!wifiScanDone) {
        wifiScanDone = true;
        printScanResult();
      }

      WiFi.begin(WIFI_SSID, WIFI_PASS);
      wifiStartMs = millis();
      wifiLastStatusMs = 0;
      wifiState = WIFI_CONNECTING;
      break;

    case WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        Serial << "Wi-Fi OK, IP: " << WiFi.localIP() << endl;
        wifiState = WIFI_CONNECTED;
        elmState = ELM_DISCONNECTED;   // запускаем TCP-цикл
      } else if (wifiConnectTimedOut()) {
        safeRestart("Wi-Fi timeout");
      } else if (wifiStatusLogDue()) {
        wifiLastStatusMs = millis();
        logWifiStatus("Wi-Fi status: ");
      }
      break;

    case WIFI_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        logWifiStatus("Wi-Fi lost, status=");
        elmClient.stop();   // безусловно: половина соединений остаётся без FIN
        elmState = ELM_DISCONNECTED;
        pollState = POLL_IDLE;
        resetPollCounters();
        // Потеря Wi-Fi не должна накапливать счётчик неудач ELM,
        // иначе короткое пропадание сети вызовет ложный мягкий сброс.
        elmFailCount = 0;
        wifiState = WIFI_DISCONNECTED;
      }
      break;
  }
}

// ======================= АВТОМАТ ELM/TCP =======================

// Адаптер ответил '?' на AT-команду (не распознал её). Не рвём соединение:
// многие клоны так отвечают, например, на ATE0, но работают дальше.
bool elmInitError() {
  String r = elmRespBuf;
  r.replace("\r", "");
  r.replace("\n", "");
  r.replace(">", "");
  r.trim();
  return elmInitStep > 0 && r == "?";
}

void elmInitSend() {
  while (elmClient.available()) elmClient.read();
  elmRespBuf = "";
  Serial << "> " << elmInitCmds[elmInitStep].cmd << endl;
  elmClient.print(elmInitCmds[elmInitStep].cmd);
  elmClient.print("\r");
  elmWaitStartMs = millis();
  elmCurrentTimeout = elmInitCmds[elmInitStep].timeout;
  elmState = ELM_INIT_WAIT;
}

void elmInitWait() {
  // Сначала вычерпываем уже пришедшие данные (может быть финальный ответ),
  // и только потом проверяем, не закрылся ли сокет.
  while (elmClient.available()) {
    char ch = (char)elmClient.read();
    elmRespBuf += ch;
    if (elmRespBuf.length() > MAX_RESP_LEN) {
      forceTcpReconnect("Response too long");
      return;
    }
    if (ch == '>') {
      Serial << "Resp: " << elmRespBuf << endl;
      if (elmInitError()) {
        // Клон ответил '?' — пропускаем эту AT-команду и идём дальше,
        // а не рвём соединение (иначе бесконечный реконнект).
        Serial << "init: '" << elmInitCmds[elmInitStep].cmd << "' -> '?', skip" << endl;
      }
      elmInitStep++;
      if (elmInitStep >= ELM_INIT_COUNT) {
        Serial << "ELM ready" << endl;
        elmState = ELM_READY;
        elmFailCount = 0;
        softResetCount = 0;       // связь восстановлена — счётчик эскалации сбрасываем
        resetPollCounters();
        // ВАЖНО: haveValidTemp/lastValidTempMs здесь НЕ трогаем.
        // Пока не получено первое валидное 0105 — мы «не взяли» управление,
        // и releaseControlTick() не должен ничего отпускать.
      } else {
        elmState = ELM_INIT_SEND;
      }
      return;
    }
  }

  if (!elmClient.connected()) {
    forceTcpReconnect("TCP lost during init");
    return;
  }

  if (elmInitTimedOut()) {
    forceTcpReconnect("ELM init timeout");
  }
}

void elmPollSend() {
  while (elmClient.available()) elmClient.read();
  pollRespBuf = "";
  elmClient.print("0105\r");
  pollSentMs = millis();
  pollState = POLL_WAIT;
}

void elmPollWait() {
  // Сначала вычерпываем уже пришедшие данные (может быть финальный ответ),
  // и только потом проверяем, не закрылся ли сокет.
  while (elmClient.available()) {
    char ch = (char)elmClient.read();
    pollRespBuf += ch;
    if (pollRespBuf.length() > MAX_RESP_LEN) {
      pollState = POLL_IDLE;
      resetPollCounters();
      forceTcpReconnect("Response too long");
      return;
    }
    if (ch == '>') {
      float t = 0;
      if (readCoolantTemp(pollRespBuf, t)) {
        Serial << "Coolant: " << t << " C" << endl;

        resetPollCounters();
        resumeOwnControl();

        haveValidTemp = true;
        lastValidTempMs = millis();
        controlRelay(t);
      } else {
        Serial << "Bad temp resp: " << pollRespBuf << endl;
        // ELM отвечает быстро, но мусором/? — таймаутов нет, поэтому
        // отдельный счётчик невалидных ответов.
        noteBadResponse();
        if (tooManyBadResponses()) {
          resetPollCounters();
          forceTcpReconnect("Too many bad responses -> force TCP reconnect");
          pollState = POLL_IDLE;
          lastPollMs = millis();
          return;
        }
      }
      pollState = POLL_IDLE;
      lastPollMs = millis();
      return;
    }
  }

  if (!elmClient.connected()) {
    pollState = POLL_IDLE;
    resetPollCounters();
    forceTcpReconnect("TCP lost during poll");
    return;
  }

  if (pollResponseTimedOut()) {
    Serial << "Poll timeout" << endl;
    pollState = POLL_IDLE;
    lastPollMs = millis();

    // Детектор «мёртвого TCP»: слишком много таймаутов подряд -> reconnect.
    notePollTimeout();
    if (tooManyPollTimeouts()) {
      resetPollCounters();
      forceTcpReconnect("Too many poll timeouts -> force TCP reconnect");
    }
  }
}

// Автомат опроса 0105 (POLL_*): отправить запрос и дождаться ответа.
void pollTick() {
  if (pollState == POLL_IDLE) {
    if (pollDue()) elmPollSend();
  } else {
    elmPollWait();
  }
}

void elmTick() {
  switch (elmState) {
    case ELM_DISCONNECTED:
      elmClient.stop();   // безусловно: гарантируем закрытие прошлого сокета
      Serial << "TCP connecting to " << ELM_IP << ":" << ELM_PORT << endl;
      // setTimeout до connect() ограничивает и время блокировки на попытке
      // соединения, чтобы не залипать в основном цикле.
      elmClient.setTimeout(1000);
      if (elmClient.connect(ELM_IP, ELM_PORT)) {
        Serial << "TCP OK" << endl;
        elmInitStep = 0;
        pollState = POLL_IDLE;
        resetPollCounters();
        elmState = ELM_INIT_SEND;
      } else {
        forceTcpReconnect("TCP connect failed");
      }
      break;

    case ELM_TCP_BACKOFF:
      if (tooManyElmFailures()) {
        // Мягкие сбросы уже не помогли — полный рестарт ESP (свежий WiFi/TCP-стек).
        if (softResetCount >= SOFT_RESETS_MAX) {
          safeRestart("ELM unrecoverable");
        }
        // Мягкий сброс Wi-Fi: дешевле полного рестарта, сохраняет гистерезис.
        Serial << "Too many ELM failures -> soft Wi-Fi reset" << endl;
        elmClient.stop();        // безусловно: отпускаем TCP-слот на адаптере
        WiFi.disconnect();       // radio остаётся включённым
        delay(200);
        pollState = POLL_IDLE;
        resetPollCounters();
        elmInitStep = 0;
        elmFailCount = 0;
        softResetCount++;
        wifiState = WIFI_DISCONNECTED;
        elmState = ELM_DISCONNECTED;
        break;
      }
      if (elmBackoffElapsed()) {
        elmState = ELM_DISCONNECTED;
      }
      break;

    case ELM_INIT_SEND:  elmInitSend(); break;
    case ELM_INIT_WAIT:  elmInitWait(); break;

    case ELM_READY:  pollTick(); break;
  }
}

// ======================= ОТДАЧА УПРАВЛЕНИЯ =======================
// Если валидной температуры нет дольше RELEASE_CONTROL_TIMEOUT_MS —
// снимаем реле и отдаём управление ЭБУ. В чужие решения не вмешиваемся.
//
// Если валидного чтения ещё не было (haveValidTemp == false),
// ничего не делаем: мы просто ещё «не взяли» управление.

void releaseControlTick() {
  if (!haveValidTemp) return;

  if (!controlReleased && noValidTempForTooLong()) {
    Serial << "!! No valid temp -> release control to ECU (relay OFF)" << endl;
    controlReleased = true;
    setRelay(false);
  }
}

// ======================= РЕЖИМ: ТОЛЬКО СКАН WI-FI =======================

#ifdef WIFI_SCAN_ONLY

void wifiScanOnlySetup() {
  Serial << "=== WIFI SCAN ONLY ===" << endl;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

// Пора выполнить очередной скан сети.
bool wifiScanDue(unsigned long lastScan) {
  return millis() - lastScan >= WIFI_SCAN_INTERVAL_MS;
}

void wifiScanOnlyLoop() {
  static unsigned long lastScan = 0;
  if (!wifiScanDue(lastScan)) return;
  lastScan = millis();

  Serial << "--- Scan ---" << endl;
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial << "  (nothing)" << endl;
    return;
  }
  for (int i = 0; i < n; i++) {
    Serial << "  " << WiFi.SSID(i)
           << "  RSSI=" << WiFi.RSSI(i)
           << "  ch=" << WiFi.channel(i)
           << (WiFi.encryptionType(i) == ENC_TYPE_NONE ? "  open" : "  enc")
           << endl;
  }
}

#endif // WIFI_SCAN_ONLY

// ======================= ЭМУЛЯТОР ТЕМПЕРАТУРЫ =======================

#ifdef DEBUG_TEMP_EMULATOR

// Плавное изменение: 80 -> 100 -> 85 -> 100 ...
// Раз в ~2 мин включается «окно сбоя» на 40 секунд — это позволяет
// проверить работу отдачи управления ЭБУ без реального железа.
float emulateTemp() {
  static float t = 80.0f;
  static int dir = 1;
  static unsigned long nextFaultMs = 90000UL;
  static unsigned long faultEndMs  = 0UL;

  unsigned long now = millis();

  if (now < faultEndMs) return NAN;   // идёт окно сбоя

  if (now >= nextFaultMs) {
    faultEndMs  = now + 40000UL;      // 40 сек без данных
    nextFaultMs = now + 120000UL;     // следующий сбой через 2 мин
    Serial << "(Emulator: 40s fault window starts)" << endl;
    return NAN;
  }

  t += 0.5f * dir;
  if (t >= 100.0f) dir = -1;
  if (t <= 85.0f)  dir =  1;
  return t;
}

void emulatorTick() {
  if (!pollDue()) return;
  lastPollMs = millis();

  float t = emulateTemp();
  if (!isnan(t)) {
    Serial << "Emulated temp: " << t << " C" << endl;
    resumeOwnControl();
    haveValidTemp = true;
    lastValidTempMs = millis();
    controlRelay(t);
  } else {
    Serial << "Emulated temp: <no data>" << endl;
    // releaseControlTick() в loop() сам отдаст управление через 30 с
  }
}

#endif // DEBUG_TEMP_EMULATOR

// ======================= НАСТРОЙКА / ЦИКЛ =======================

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial << endl;

#if defined(DEBUG_TEMP_EMULATOR)
  Serial << "=== ESP8266 car fan — DEBUG TEMP EMULATOR ===" << endl;
#elif defined(WIFI_SCAN_ONLY)
  // заголовок печатает wifiScanOnlySetup()
#else
  Serial << "=== ESP8266 car fan controller (ECU backup) ===" << endl;
#endif

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  relayState = false;
  controlReleased = false;
  haveValidTemp = false;

  // Резервируем буферы String, чтобы не фрагментировать heap при долгой работе.
  // (особенно важно на ESP8266, где String — известный источник утечек)
  elmRespBuf.reserve(160);
  pollRespBuf.reserve(96);

#if defined(DEBUG_TEMP_EMULATOR)
  haveValidTemp = true;
  lastValidTempMs = millis();
#elif defined(WIFI_SCAN_ONLY)
  wifiScanOnlySetup();
#else
  // Разовая настройка Wi-Fi (в рабочем режиме повторно не требуется).
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  wifiState = WIFI_DISCONNECTED;
#endif
}

void loop() {
#if defined(DEBUG_TEMP_EMULATOR)
  emulatorTick();
#elif defined(WIFI_SCAN_ONLY)
  wifiScanOnlyLoop();
  return;               // в этом режиме ничего больше не делаем
#else
  wifiTick();
  if (wifiState == WIFI_CONNECTED) {
    elmTick();
  }
#endif
  releaseControlTick();
  yield();
}