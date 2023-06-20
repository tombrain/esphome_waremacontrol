#pragma once

#include "esphome.h"

#include "RCSwitch.h"
#include <ArduinoJson.h>
/*
{
  "codedevice_first": "S01110100101110",
  "codedevice_second": "S000011010",
  "code_action": "S011110101S",
  "data_length": 1780,
  "sync_length": 5000,
  "num_commands": 3,
  "send_delay": 10000
}
*/
class RCSwitchWarema : public RCSwitch
{
	using Base=RCSwitch;

	public:

	void sendMC(std::string const& sCodeWord, unsigned int dataLength, unsigned int syncLength, uint8_t sendCommand, unsigned int sendDelay  )
	{
		auto const halfDataLength{dataLength / 2};
		for (uint8_t nRepeat{0}; nRepeat < sendCommand; ++nRepeat)
		{
			digitalWrite(m_transmitterPin, LOW);

			for (auto const& c : sCodeWord)
			{
				switch (c)
				{
					case 's':
						digitalWrite(m_transmitterPin, LOW);
						delayMicroseconds(syncLength);
						break;

					case 'S':
						digitalWrite(m_transmitterPin, HIGH);
						delayMicroseconds(syncLength);
						break;

					case '0':
						digitalWrite(m_transmitterPin, HIGH);
						delayMicroseconds(halfDataLength);
						digitalWrite(m_transmitterPin, LOW);
						delayMicroseconds(halfDataLength);
						break;

					case '1':
						digitalWrite(m_transmitterPin, LOW);
						delayMicroseconds(halfDataLength);
						digitalWrite(m_transmitterPin, HIGH);
						delayMicroseconds(halfDataLength);
						break;
				}
			}
			digitalWrite(this->m_transmitterPin, LOW);
			if (nRepeat != sendCommand - 1)
			{
				delayMicroseconds(sendDelay);
			}
		}
	}
	
	void enableTransmit(int nTransmitterPin)
	{
		m_transmitterPin = nTransmitterPin;

		Base::enableTransmit(nTransmitterPin);
	}

private:
	int m_transmitterPin{};
};


class WaremaBlindComponent : public Component, public CustomMQTTDevice
{
public:
	WaremaBlindComponent(uint8_t transmitterPin, std::string const& topicBase)
	: m_transmitterPin{transmitterPin}
	, m_topicBase{topicBase}
	{
	}

	void setup() override
	{
		{
			std::string const t{m_topicBase + "/command/waremablind"};
			subscribe(t, &WaremaBlindComponent::on_command_waremasend);
		}
		{
			std::string const t{m_topicBase + "/command/waremablind_json"};
			subscribe(t, &WaremaBlindComponent::on_command_waremasend_json);
		}
		{
			std::string const t{m_topicBase + "/command/raw433"};
			subscribe(t, &WaremaBlindComponent::on_command_raw433);
		}

		ESP_LOGD("custom", "WaremaBlindComponent setup: baseTopic '%s'", m_topicBase.c_str());
	}
  
	void on_command_waremasend(const std::string &payload)
	{
		ESP_LOGD("custom", "on_command_waremasend payload: '%s'", payload.c_str());

		if(payload.length() == 36)
		{
			m_mySwitch.enableTransmit(m_transmitterPin);
			m_mySwitch.sendMC(payload,m_dataLength,m_syncLength,m_sendCommand,m_sendDelay);
			m_mySwitch.disableTransmit();
		}
	}
  
	void on_command_waremasend_json(const std::string &payload)
	{
		ESP_LOGD("custom", "on_command_waremasend_json payload: '%s'", payload.c_str());
		
		StaticJsonDocument<300> doc;
		
		// Deserialize the JSON document
		DeserializationError error = deserializeJson(doc, payload);
		
		if (error)
		{
			ESP_LOGD("custom", "on_command_waremasend_json payload: deserializeJson() failed: '%s'", error.f_str());
		}
		else
		{
			ESP_LOGD("custom", "on_command_waremasend_json: start");
			const char* codedevice_first{doc["codedevice_first"]};
			ESP_LOGD("custom", "on_command_waremasend_json payload: codedevice_first '%s'", codedevice_first);
			const char* codedevice_second{doc["codedevice_second"]};
			ESP_LOGD("custom", "on_command_waremasend_json payload: codedevice_second '%s'", codedevice_second);
			const char* code_action{doc["code_action"]};
			ESP_LOGD("custom", "on_command_waremasend_json payload: code_action '%s'", code_action);

			unsigned int const data_length{doc["data_length"]};
			ESP_LOGD("custom", "on_command_waremasend_json payload: data_length '%d'", data_length);
			if (data_length)
			{
				m_dataLength = data_length;
			}
			ESP_LOGD("custom", "on_command_waremasend_json: m_dataLength '%d'", m_dataLength);
			unsigned int const sync_length{doc["sync_length"]};
			ESP_LOGD("custom", "on_command_waremasend_json payload: sync_length '%d'", sync_length);
			if (sync_length)
			{
				m_syncLength = sync_length;
			}
			ESP_LOGD("custom", "on_command_waremasend_json: m_syncLength '%d'", m_syncLength);
			uint8_t const num_commands{doc["num_commands"]};
			ESP_LOGD("custom", "on_command_waremasend_json payload: num_commands '%d'", num_commands);
			if (num_commands)
			{
				m_sendCommand = num_commands;
			}
			ESP_LOGD("custom", "on_command_waremasend_json: m_sendCommand '%d'", m_sendCommand);
			unsigned int const send_delay{doc["send_delay"]};
			ESP_LOGD("custom", "on_command_waremasend_json payload: send_delay '%d'", send_delay);
			if (send_delay)
			{
				m_sendDelay = send_delay;
			}
			ESP_LOGD("custom", "on_command_waremasend_json: m_sendDelay '%d'", m_sendDelay);

			std::string const command{std::string{codedevice_first} + std::string{codedevice_second} + std::string{code_action} + "S"};
			
			ESP_LOGD("custom", "on_command_waremasend_json command: '%s'", command.c_str());
			if(command.length() == 36)
			{
				m_mySwitch.enableTransmit(m_transmitterPin);
				m_mySwitch.sendMC(command,m_dataLength,m_syncLength,m_sendCommand,m_sendDelay);
				m_mySwitch.disableTransmit();
			}
		}
	}
  
	void on_command_raw433(const std::string &payload)
	{
		ESP_LOGD("custom", "on_command_waremasend payload: '%s'", payload.c_str());

		m_mySwitch.enableTransmit(m_transmitterPin);
		m_mySwitch.send(payload.c_str());
		m_mySwitch.disableTransmit();
	}
  
	void on_json_message(JsonObject &root)
	{
		if (!root.containsKey("key"))
			return;

		int value = root["key"];
		// do something with Json Object

		// publish JSON using lambda syntax
		//publish_json("the/other/json/topic", [=](JsonObject &root2) {
		//  root2["key"] = "Hello World";
		//});
	}
  
private:
	unsigned int m_dataLength{1780};
	unsigned int m_syncLength{5000};
	uint8_t m_sendCommand{3};
	unsigned int m_sendDelay{10000};

	uint8_t m_transmitterPin{};
	std::string const m_topicBase{};
	RCSwitchWarema m_mySwitch{};

};