#include <Arduino.h>
#include <myIOT2.h>

#define TURN_ON HIGH
#define TURN_OFF LOW
#define BLINK_INTERVAL 500

myIOT2 iot;

enum AlarmState_Code
{
  DISARMED,
  ARMED_HOME,
  ARMED_AWAY,
  PENDNG,
  TRIGGERED
};

AlarmState_Code current_alarm_state_code = DISARMED;
AlarmState_Code previous_alarm_state_code = DISARMED;

bool alarm_avail = false;
uint8_t led_pins[] = {D1, D2, D2, D2, D3}; // ARMED_HOME, ARMED_AWAY, and PNDING share the same LED

unsigned long lastBlinkTime = 0;

char alarm_topics[2][50];
const char *param_filenames[] = {"iot_params.JSON", "iot_topics.JSON", "app_params.JSON"};

constexpr const char *alarm_avail_states[] = {"online", "offline"};
constexpr const char *alarm_states[] = {"disarmed", "armed_home", "armed_away", "pending", "triggered"};

// ~~~~~~~~~~~~ iot2 ~~~~~~~~~~~~
void extMQTT(char *incoming_msg, char *_topic)
{
  char msg[270];

  if (strcmp(incoming_msg, "status") == 0)
  {
    sprintf(msg, "[Status]: State[%s], Avail[%s]",
            alarm_states[current_alarm_state_code], alarm_avail ? alarm_avail_states[0] : alarm_avail_states[1]);
    iot.pub_msg(msg);
  }
  if (strcmp(_topic, alarm_topics[0]) == 0)
  {
    if (strcmp(incoming_msg, alarm_avail_states[0]) == 0)
    {
      alarm_avail = true;
    }
    else if (strcmp(incoming_msg, alarm_avail_states[1]) == 0)
    {
      alarm_avail = false;
    }
  }
  if (strcmp(_topic, alarm_topics[1]) == 0)
  {
    uint8_t i = 0;
    for (const char *state : alarm_states)
    {
      if (strcmp(incoming_msg, state) == 0)
      {
        current_alarm_state_code = AlarmState_Code(i);
        // create_log_emtry(state);
        sprintf(msg, "[Alarm Monitor]: %s", state);
        Serial.println(msg);
      }
      i++;
    }
  }
}
void set_hardcoded_topics()
{
  const char *default_subTopics[] = {"DvirHome/device1", "DvirHome/All"};
  const char *default_pubTopics[] = {"DvirHome/device1/Avail", "DvirHome/device1/State"};
  const char *default_gen_pubTopics[] = {"DvirHome/Messages", "DvirHome/log", "DvirHome/debug"};

  iot.add_subTopic(default_subTopics, sizeof(default_subTopics) / sizeof(default_subTopics[0]));
  iot.add_pubTopic(default_pubTopics, sizeof(default_pubTopics) / sizeof(default_pubTopics[0]));
  iot.add_gen_pubTopic(default_gen_pubTopics, sizeof(default_gen_pubTopics) / sizeof(default_gen_pubTopics[0]));
}
void set_topics_from_flash(JsonDocument &DOC)
{
  JsonArray subTopics = DOC["subTopic"].as<JsonArray>();
  JsonArray pubTopic = DOC["pubTopic"].as<JsonArray>();
  JsonArray gen_pubTopic = DOC["gen_pubTopic"].as<JsonArray>();

  for (const auto &topic : subTopics)
  {
    iot.add_subTopic(topic);
  }
  for (const auto &topic : pubTopic)
  {
    iot.add_pubTopic(topic);
  }
  for (const auto &topic : gen_pubTopic)
  {
    iot.add_gen_pubTopic(topic);
  }
}
void start_iot2()
{
  StaticJsonDocument<600> DOC;

  iot.set_pFilenames(param_filenames, sizeof(param_filenames) / sizeof(param_filenames[0])); // set the filenames for the parameters
  iot.readFlashParameters(DOC, param_filenames[0]);                                          // iot2 Parameters. in case of failure, default values will be used

  if (iot.readJson_inFlash(DOC, param_filenames[1])) // Topics. read topics from flash or use hardcoded topics
  {
    set_topics_from_flash(DOC);
  }
  else
  {
    set_hardcoded_topics();
  }

  if (iot.readJson_inFlash(DOC, param_filenames[2]))
  {
    for (uint8_t i = 0; i < DOC["output_pins"].size(); i++)
    {
      led_pins[i] = DOC["output_pins"][i].as<uint8_t>();
    }

    for (uint8_t i = 0; i < DOC["sub_Topics"].size(); i++)
    {
      iot.add_subTopic(DOC["sub_Topics"][i].as<const char *>());
      strcpy(alarm_topics[i], DOC["sub_Topics"][i].as<const char *>());
    }
  }

  iot.start_services(extMQTT);
}

// ~~~~~~~~~~~~ led output ~~~~~~~~~~~~
void init_output()
{
  for (uint8_t pin : led_pins)
  {
    pinMode(pin, OUTPUT);
  }
}
void test_print(const char *msg)
{
  Serial.print(iot.now());
  Serial.println(msg);
}
void turn_leds_off()
{
  for (uint8_t pin : led_pins)
  {
    digitalWrite(pin, TURN_OFF);
  }
}
void alarm_state_looper()
{
  unsigned long currentMillis = millis();
  if (currentMillis - lastBlinkTime >= BLINK_INTERVAL)
  {
    lastBlinkTime = currentMillis;
    if (previous_alarm_state_code != current_alarm_state_code)
    {
      digitalWrite(led_pins[previous_alarm_state_code], TURN_OFF);
    }
    digitalWrite(led_pins[current_alarm_state_code], !digitalRead(led_pins[current_alarm_state_code]));
    previous_alarm_state_code = current_alarm_state_code;
  }
}

void setup()
{
  init_output();
  start_iot2();
}

void loop()
{
  iot.looper();
  alarm_state_looper();
}