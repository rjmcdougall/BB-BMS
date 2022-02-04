//#include <TelnetStream.h>
#include "bms_can.h"
#include "crc16.h"
#include "buffer.h"

#define CAN_DEBUG 1

bms_can::bms_can(diybms_eeprom_settings *s, CellModuleInfo *c, PackInfo *p)
{
	bms_can::settings = s;
	//ESP_LOGD(TAG, "Initializing bms:series modules = %d", settings->totalNumberOfSeriesModules);
	//ESP_LOGD(TAG, "Initializing bms:series modules = %d", settings->controller_id);
	settings->totalNumberOfSeriesModules = 12;
	settings->controller_id = 10;
	bms_can::cmi = c;
	bms_can::pi = p;
}

void bms_can::begin(void)
{
	if (CAN_DEBUG)
	{
		ESP_LOGD(TAG, "Initializing bms_can...");
	}
	ESP_LOGD(TAG, "Initializing bms:series modules = %d", settings->totalNumberOfSeriesModules);
	ESP_LOGD(TAG, "Initializing bms:controller id = %d", settings->controller_id);
	initCAN();
}

TaskHandle_t bms_can::can_read_task_handle = nullptr;
TaskHandle_t bms_can::can_process_task_handle = nullptr;
TaskHandle_t bms_can::can_status_task_handle = nullptr;
QueueHandle_t bms_can::queue_canrx = nullptr;
QueueHandle_t bms_can::queue_ping = nullptr;

can_status_msg bms_can::stat_msgs[CAN_STATUS_MSGS_TO_STORE];
can_status_msg_2 bms_can::stat_msgs_2[CAN_STATUS_MSGS_TO_STORE];
can_status_msg_3 bms_can::stat_msgs_3[CAN_STATUS_MSGS_TO_STORE];
can_status_msg_4 bms_can::stat_msgs_4[CAN_STATUS_MSGS_TO_STORE];
can_status_msg_5 bms_can::stat_msgs_5[CAN_STATUS_MSGS_TO_STORE];
bms_soc_soh_temp_stat bms_can::bms_stat_msgs[CAN_BMS_STATUS_MSGS_TO_STORE];
bms_soc_soh_temp_stat bms_can::bms_stat_v_cell_min;

diybms_eeprom_settings *bms_can::settings = nullptr;
CellModuleInfo *bms_can::cmi = nullptr;
PackInfo *bms_can::pi = nullptr;
unsigned int bms_can::rx_buffer_last_id = -1;

int bms_can::rx_count = 0;
int bms_can::tx_count = 0;

static void send_packet_wrapper(unsigned char *data, unsigned int len);

void bms_can::can_read_task_static(void *param)
{
	static_cast<bms_can *>(param)->can_read_task();
}

void bms_can::can_process_task_static(void *param)
{
	static_cast<bms_can *>(param)->can_process_task();
}

void bms_can::can_status_task_static(void *param)
{
	static_cast<bms_can *>(param)->can_status_task();
}

void enable_alerts()
{
	// Reconfigure alerts to detect Error Passive and Bus-Off error states

	uint32_t alerts_to_enable = CAN_ALERT_ERR_PASS | CAN_ALERT_BUS_OFF;
	if (can_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
	{
		ESP_LOGI(TAG, "Alerts reconfigured\n");
	}
	else
	{
		ESP_LOGI(TAG, "Failed to reconfigure alerts");
	}
}

void bms_can::initCAN()
{
	for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
	{
		stat_msgs[i].id = -1;
		stat_msgs_2[i].id = -1;
		stat_msgs_3[i].id = -1;
		stat_msgs_4[i].id = -1;
		stat_msgs_5[i].id = -1;
	}

	bms_stat_v_cell_min.id = -1;

	for (int i = 0; i < CAN_BMS_STATUS_MSGS_TO_STORE; i++)
	{
		bms_stat_msgs[i].id = -1;
	}

	//xTaskCreate(bms_can::can_read_task_static, "bms_can", 3000, nullptr, 1, &bms_can::can_task_handle);
	//Task can_task = new Task("can_task", can_read_task, 1, 16);
	// Initialize configuration structures using macro initializers
	//can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_5, gpio_num_t::GPIO_NUM_4, CAN_MODE_LISTEN_ONLY);
	//can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_5, gpio_num_t::GPIO_NUM_35, CAN_MODE_NORMAL);
	//can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_13, gpio_num_t::GPIO_NUM_35, CAN_MODE_NORMAL);
	//can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_13, gpio_num_t::GPIO_NUM_36, CAN_MODE_NORMAL);
	can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_26, gpio_num_t::GPIO_NUM_36, CAN_MODE_NORMAL);
	//g_config.mode = CAN_MODE_NORMAL;
	can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
	//can_timing_config_t t_config = CAN_TIMING_CONFIG_50KBITS();
	can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
	//pinMode(GPIO_NUM_32, INPUT_PULLDOWN);

	//Install CAN driver
	if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
	{
		ESP_LOGI(TAG, "CAN Driver installed\n");
	}
	else
	{
		ESP_LOGI(TAG, "Failed to install CAN driver\n");
	}

	// Let CANBUS start
	//vTaskDelay(pdMS_TO_TICKS(3000));

	enable_alerts();

	//Start CAN driver
	if (can_start() == ESP_OK)
	{
		ESP_LOGI(TAG, "CAN Driver started\n");
	}
	else
	{
		ESP_LOGI(TAG, "Failed to start driver\n");
	}

	// TODO: move these to init only once
	ESP_LOGI(TAG, "CAN  init task queue_canrx = %lx\n", &bms_can::queue_canrx);
	xTaskCreate(bms_can::can_read_task_static, "bms_can_read", 8000, nullptr, 1, nullptr);
	xTaskCreate(bms_can::can_process_task_static, "bms_can_process", 8000, nullptr, 1, nullptr);
	//vTaskDelay(pdMS_TO_TICKS(5000));
	xTaskCreate(bms_can::can_status_task_static, "bms_can_status", 8000, nullptr, 1, nullptr);
}

// TODO: For power management
void bms_can::sleep_reset()
{
}

//Switch CANBUS off, saves a couple of milliamps
//hal.CANBUSEnable(false);

void bms_can::can_read_task()
{
	vTaskDelay(pdMS_TO_TICKS(2000));
	ESP_LOGI(TAG, "CAN  starting read task\n");
	bms_can::queue_canrx = xQueueCreate(10, sizeof(can_message_t));
	ESP_LOGI(TAG, "CAN  read task queue_canrx = %lx\n", &bms_can::queue_canrx);
	if (bms_can::queue_canrx == nullptr)
	{
		ESP_LOGI(TAG, "CAN  null queue\n");
		return;
	}
	can_message_t message;
	while (1)
	{

		//Wait for message to be received
		esp_err_t res = can_receive(&message, pdMS_TO_TICKS(8000));

		if (res == ESP_OK)
		{
			//ESP_LOGI(TAG, "CAN Message received id = %d\n", message.identifier);
			if (!(message.flags & CAN_MSG_FLAG_RTR))
			{
				xQueueSendToBack(bms_can::queue_canrx, &message, 16);
				rx_count++;
			}
		}
		else if (res == ESP_ERR_TIMEOUT)
		{
			/// ignore the timeout or do something
			//ESP_LOGI(TAG, "Timeout");
		}
	}
}

void bms_can::can_process_task()
{
	vTaskDelay(pdMS_TO_TICKS(2000));
	ESP_LOGI(TAG, "CAN  starting process task\n");
	ESP_LOGI(TAG, "CAN  process task queue_canrx = %lx\n", &bms_can::queue_canrx);
	ESP_LOGI(TAG, "CAN  process task queue_canrx contents = %lx\n", bms_can::queue_canrx);
	can_message_t rxmsg;
	for (;;)
	{
		if (xQueueReceive(bms_can::queue_canrx, &rxmsg, portMAX_DELAY) == pdPASS)
		{
			//ESP_LOGI(TAG, "CAN  process got msg %x %x %d!\n", rxmsg.identifier, rxmsg.flags, rxmsg.data_length_code);
			if (rxmsg.flags == CAN_MSG_FLAG_EXTD)
			{
				decode_msg(rxmsg.identifier, &rxmsg.data[0], rxmsg.data_length_code, false);
			}
			else
			{
				//if (sid_callback) {
				//	sid_callback(rxmsg.SID, rxmsg.data8, rxmsg.data_length_code);
				//}
			}
		}
	}
}

void commands_send_packet(unsigned char *data, unsigned int len)
{
	ESP_LOGI(TAG, "CAN  send packet: \n");
}

void bms_can::can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send)
{
	uint8_t send_buffer[8];

	if (len <= 6)
	{
		uint32_t ind = 0;
		send_buffer[ind++] = settings->controller_id;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		can_transmit_eid(controller_id |
							 ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8),
						 send_buffer, ind);
	}
	else
	{
		unsigned int end_a = 0;
		for (unsigned int i = 0; i < len; i += 7)
		{
			if (i > 255)
			{
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len)
			{
				memcpy(send_buffer + 1, data + i, send_len);
			}
			else
			{
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			can_transmit_eid(controller_id |
								 ((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8),
							 send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a; i < len; i += 6)
		{
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len)
			{
				memcpy(send_buffer + 2, data + i, send_len);
			}
			else
			{
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			can_transmit_eid(controller_id |
								 ((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8),
							 send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = settings->controller_id;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = CRC16::CalculateArray(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		can_transmit_eid(controller_id |
							 ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8),
						 send_buffer, ind++);
	}
}

//void commands_process_packet(unsigned char *data, unsigned int len, void(*reply_func)(unsigned char *data, unsigned int len)
void bms_can::commands_process_packet(unsigned char *data, unsigned int len)
{
	//	if (len < 1 || reply_func == 0) {
	if (len < 1)
	{
		return;
	}

	//send_func = reply_func;

	int packet_id = data[0];
	data++;
	len--;
	char buffer[128];
	ESP_LOGI(TAG, "CAN  process packet: %d\n", packet_id);

	sprintf(buffer, "CAN  command %x\n", packet_id);

	//	TelnetStream.println(buffer);

	switch (packet_id)
	{
	case COMM_FW_VERSION:
	{
		ESP_LOGI(TAG, "CAN  process packet: COMM_FW_VERSION\n");

		int32_t ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;

		strcpy((char *)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;

		uint64_t esp_chip_id[2];
		esp_chip_id[0] = ESP.getEfuseMac();

		memcpy(send_buffer + ind, &esp_chip_id[0], 12);
		ind += 12;

		send_buffer[ind++] = 0;
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;

		send_buffer[ind++] = HW_TYPE_VESC_BMS;

		send_buffer[ind++] = 1; // No custom config

		//reply_func(send_buffer, ind);
		can_send_buffer(rx_buffer_last_id, send_buffer, len, 1);
	}
	break;

	default:
		break;
	}
}

//static void send_packet_wrapper(unsigned char *data, unsigned int len) {
//	can_send_buffer(rx_buffer_last_id, data, len, 1);
//}

void bms_can::decode_msg(uint32_t eid, uint8_t *data8, int len, bool is_replaced)
{
	int32_t ind = 0;
	unsigned int rxbuf_len;
	unsigned int rxbuf_ind;
	uint8_t crc_low;
	uint8_t crc_high;
	uint8_t commands_send;

	uint8_t id = eid & 0xFF;
	int32_t cmd = eid >> 8;
	//ESP_LOGI(TAG, "CAN  (i am %x) decode from id %x cmd %x d0=%x\n", settings->controller_id, id, cmd, data8[0]);
	char buffer[128];
	sprintf(buffer, "CAN  %x decode id %x cmd %x\n", settings->controller_id, id, cmd);
	//TelnetStream.println(buffer);

	if (id == 255 || id == settings->controller_id)
	//	if (id == 255 || id == 99)
	{
		ESP_LOGI(TAG, "CAN  decode 255 or me: id %x cmd %d len %d\n", id, cmd, len);
		//sprintf(buffer, "CAN  decode 255 or me id %x cmd %x\n", id, cmd);
		//TelnetStream.println(buffer);
		switch (cmd)
		{
		case CAN_PACKET_FILL_RX_BUFFER:
			ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_FILL_RX_BUFFER\n");
			memcpy(rx_buffer + data8[0], data8 + 1, len - 1);
			break;

		case CAN_PACKET_FILL_RX_BUFFER_LONG:
			ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_FILL_RX_BUFFER_LONG\n");
			rxbuf_ind = (unsigned int)data8[0] << 8;
			rxbuf_ind |= data8[1];
			if (rxbuf_ind < RX_BUFFER_SIZE)
			{
				memcpy(rx_buffer + rxbuf_ind, data8 + 2, len - 2);
			}
			break;

		case CAN_PACKET_PROCESS_RX_BUFFER:
			ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_PROCESS_RX_BUFFER id %x cmd %d, len %d\n", id, cmd, len);
			sprintf(buffer, "CAN  CAN_PACKET_PROCESS_RX_BUFFER 		id %x cmd %d\n", id, cmd);
			//			TelnetStream.println(buffer);
			//			TelnetStream.flush();
			ind = 0;
			rx_buffer_last_id = data8[ind++];
			commands_send = data8[ind++];
			rxbuf_len = (unsigned int)data8[ind++] << 8;
			rxbuf_len |= (unsigned int)data8[ind++];

			if (rxbuf_len > RX_BUFFER_SIZE)
			{
				break;
			}

			break;

			crc_high = data8[ind++];
			crc_low = data8[ind++];

			if (CRC16::CalculateArray(rx_buffer, rxbuf_len) == ((unsigned short)crc_high << 8 | (unsigned short)crc_low))
			{

				if (is_replaced)
				{
					if (rx_buffer[0] == COMM_JUMP_TO_BOOTLOADER ||
						rx_buffer[0] == COMM_ERASE_NEW_APP ||
						rx_buffer[0] == COMM_WRITE_NEW_APP_DATA ||
						rx_buffer[0] == COMM_WRITE_NEW_APP_DATA_LZO ||
						rx_buffer[0] == COMM_ERASE_BOOTLOADER)
					{
						break;
					}
				}

				sleep_reset();

				switch (commands_send)
				{
				case 0:
					//TelnetStream.println("commands_process_packet");
					//TelnetStream.flush();
					//commands_process_packet(rx_buffer, rxbuf_len, send_packet_wrapper);
					commands_process_packet(rx_buffer, rxbuf_len);
					break;
				case 1:
					//TelnetStream.println("commands_send_packet");
					//TelnetStream.flush();
					commands_send_packet(rx_buffer, rxbuf_len);
					break;
				case 2:
					//TelnetStream.println("commands_process_packet");
					//TelnetStream.flush();
					//commands_process_packet(rx_buffer, rxbuf_len, 0);
					commands_process_packet(rx_buffer, rxbuf_len);
					break;
				default:
					break;
				}
			}
			break;

		case CAN_PACKET_PROCESS_SHORT_BUFFER:

			ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_PROCESS_SHORT_BUFFER id %x cmd %x  len %d\n", id, cmd, len);

			ind = 0;
			rx_buffer_last_id = data8[ind++];
			commands_send = data8[ind++];

			ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_PROCESS_SHORT_BUFFER id %x cmd %x  cmd_send %x len %d\n", id, cmd, commands_send, len);
			//sprintf(buffer, "CAN  CAN_PACKET_PROCESS_SHORT_BUFFER 		id %x cmd %x cmd_send %x\n", id, cmd, commands_send);
			//TelnetStream.println(buffer);
			//TelnetStream.flush();

			if (is_replaced)
			{
				if (data8[ind] == COMM_JUMP_TO_BOOTLOADER ||
					data8[ind] == COMM_ERASE_NEW_APP ||
					data8[ind] == COMM_WRITE_NEW_APP_DATA ||
					data8[ind] == COMM_WRITE_NEW_APP_DATA_LZO ||
					data8[ind] == COMM_ERASE_BOOTLOADER)
				{
					break;
				}
			}

			switch (commands_send)
			{
			case 0:
				//TelnetStream.println("commands_process_packet");
				//TelnetStream.flush();
				//commands_process_packet(data8 + ind, len - ind, send_packet_wrapper);
				commands_process_packet(data8 + ind, len - ind);
				break;
			case 1:
				//TelnetStream.println("commands_send_packet");
				//TelnetStream.flush();
				commands_send_packet(data8 + ind, len - ind);
				break;
			case 2:
				//TelnetStream.println("commands_process_packet");
				//TelnetStream.flush();
				//commands_process_packet(data8 + ind, len - ind, 0);
				commands_process_packet(data8 + ind, len - ind);
				break;
			default:
				break;
			}
			break;

		case CAN_PACKET_PING:
			ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_PING\n");
			//TelnetStream.println("ping");
			//TelnetStream.flush();
			{
				uint8_t buffer[2];
				buffer[0] = settings->controller_id;
				buffer[1] = HW_TYPE_VESC_BMS;
				can_transmit_eid(data8[0] |
									 ((uint32_t)CAN_PACKET_PONG << 8),
								 buffer, 2);
			}
			break;

		case CAN_PACKET_PONG:
			ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_PONG\n");
			// data8[0]; // Sender ID
			if (queue_ping)
			{
				if (len >= 2)
				{
					ping_hw_last = (HW_TYPE)data8[1];
				}
				else
				{
					ping_hw_last = HW_TYPE_VESC_BMS;
				}
				// TODO: convert ping_hw_last to a message that is passed
				uint32_t dummy_msg = 0;
				//xQueueSendToBack(queue_ping, &dummy_msg, 0);
			}
			break;

		case CAN_PACKET_SHUTDOWN:
		{
			// TODO: Implement when hw has power switch
		}
		break;

		default:
			break;
		}
	}

	switch (cmd)
	{
	case CAN_PACKET_PING:
		ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_PING\n");
		sleep_reset();
		break;

	case CAN_PACKET_STATUS:
		//ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_STATUS\n");
		sleep_reset();

		for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
		{
			//ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_STATUS %d\n", i);
			can_status_msg *stat_tmp = &stat_msgs[i];
			//ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_STATUS %d id = %d\n", i, stat_tmp->id);
			if (stat_tmp->id == id || stat_tmp->id == -1)
			{
				ind = 0;
				stat_tmp->id = id;
				stat_tmp->rx_time = millis();
				stat_tmp->rpm = (float)buffer_get_int32(data8, &ind);
				stat_tmp->current = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp->duty = (float)buffer_get_int16(data8, &ind) / 1000.0;
#define BAJA_HEADLIGHT_PIN GPIO_NUM_13
				if (stat_tmp->rpm > 0)
				{
					//digitalWrite(BAJA_HEADLIGHT_PIN, LOW);
					//TelnetStream.println("h/l on");
					//TelnetStream.flush();
				}
				else
				{
					//TelnetStream.println("h/l off");
					//TelnetStream.flush();
					//digitalWrite(BAJA_HEADLIGHT_PIN, HIGH);
				}
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_2:
		//ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_STATUS2\n");
		for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
		{
			can_status_msg_2 *stat_tmp_2 = &stat_msgs_2[i];
			if (stat_tmp_2->id == id || stat_tmp_2->id == -1)
			{
				ind = 0;
				stat_tmp_2->id = id;
				stat_tmp_2->rx_time = millis();
				stat_tmp_2->amp_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				stat_tmp_2->amp_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_3:
		//ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_STATUS3\n");
		for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
		{
			can_status_msg_3 *stat_tmp_3 = &stat_msgs_3[i];
			if (stat_tmp_3->id == id || stat_tmp_3->id == -1)
			{
				ind = 0;
				stat_tmp_3->id = id;
				stat_tmp_3->rx_time = millis();
				stat_tmp_3->watt_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				stat_tmp_3->watt_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_4:
		//ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_STATUS4\n");
		for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
		{
			can_status_msg_4 *stat_tmp_4 = &stat_msgs_4[i];
			if (stat_tmp_4->id == id || stat_tmp_4->id == -1)
			{
				ind = 0;
				stat_tmp_4->id = id;
				stat_tmp_4->rx_time = millis();
				stat_tmp_4->temp_fet = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->temp_motor = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->current_in = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->pid_pos_now = (float)buffer_get_int16(data8, &ind) / 50.0;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_5:
		//ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_STATUS5\n");
		for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
		{
			can_status_msg_5 *stat_tmp_5 = &stat_msgs_5[i];
			if (stat_tmp_5->id == id || stat_tmp_5->id == -1)
			{
				ind = 0;
				stat_tmp_5->id = id;
				stat_tmp_5->rx_time = millis();
				stat_tmp_5->tacho_value = buffer_get_int32(data8, &ind);
				stat_tmp_5->v_in = (float)buffer_get_int16(data8, &ind) / 1e1;
				ESP_LOGI(TAG, "CAN  decode: v_in = %f\n", stat_tmp_5->v_in);
				break;
			}
		}
		break;

	case CAN_PACKET_BMS_SOC_SOH_TEMP_STAT:
		//	ESP_LOGI(TAG, "CAN  decode: CAN_PACKET_BMS_SOC_SOH_TEMP_STAT\n");
		{
			int32_t ind = 0;
			bms_soc_soh_temp_stat msg;
			msg.id = id;
			msg.rx_time = millis();
			msg.v_cell_min = buffer_get_float16(data8, 1e3, &ind);
			msg.v_cell_max = buffer_get_float16(data8, 1e3, &ind);
			msg.soc = ((float)((uint8_t)data8[ind++])) / 255.0;
			msg.soh = ((float)((uint8_t)data8[ind++])) / 255.0;
			msg.t_cell_max = (float)((int8_t)data8[ind++]);
			uint8_t stat = data8[ind++];
			msg.is_charging = (stat >> 0) & 1;
			msg.is_balancing = (stat >> 1) & 1;
			msg.is_charge_allowed = (stat >> 2) & 1;

			// Do not go to sleep when some other pack is charging or balancing.
			if (msg.is_charging || msg.is_balancing)
			{
				sleep_reset();
			}

			// Find BMS with highest cell voltage
			if (bms_stat_v_cell_min.id < 0 ||
				UTILS_AGE_S(bms_stat_v_cell_min.rx_time) > 2.0 ||
				bms_stat_v_cell_min.v_cell_min > msg.v_cell_min)
			{
				bms_stat_v_cell_min = msg;
			}
			else if (bms_stat_v_cell_min.id == msg.id)
			{
				bms_stat_v_cell_min = msg;
			}

			for (int i = 0; i < CAN_BMS_STATUS_MSGS_TO_STORE; i++)
			{
				bms_soc_soh_temp_stat *msg_buf = &bms_stat_msgs[i];

				// Reset ID after 10 minutes of silence
				if (msg_buf->id != -1 && UTILS_AGE_S(msg_buf->rx_time) > 60 * 10)
				{
					msg_buf->id = -1;
				}

				if (msg_buf->id == id || msg_buf->id == -1)
				{
					*msg_buf = msg;
					break;
				}
			}
		}
		break;

	default:
		//ESP_LOGI(TAG, "CAN  decode: default\n");
		break;
	}
}

/**
 * Check if a VESC on the CAN-bus responds.
 *
 * @param controller_id
 * The ID of the VESC.
 *
 * @param hw_type
 * The hardware type of the CAN device.
 *
 * @return
 * True for success, false otherwise.
 */
bool bms_can::can_ping(uint8_t controller_id, HW_TYPE *hw_type)
{

	queue_ping = xQueueCreate(10, sizeof(int32_t));
	int32_t ping_msg;
	uint8_t buffer[1];
	buffer[0] = settings->controller_id;
	can_transmit_eid(controller_id |
						 ((uint32_t)CAN_PACKET_PING << 8),
					 buffer, 1);

	int ret = xQueueReceive(queue_ping, &ping_msg, portMAX_DELAY);

	vQueueDelete(queue_ping);
	queue_ping = NULL;

	if (ret = pdPASS)
	{
		if (hw_type)
		{
			*hw_type = ping_hw_last;
		}
	}

	return ret != 0;
}

void bms_can::can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len)
{
	if (len > 8)
	{
		len = 8;
	}

	can_message_t txmsg;
	//txmsg. = CAN_IDE_EXT;
	txmsg.identifier = id;
	txmsg.flags = CAN_MSG_FLAG_EXTD;
	//txmsg.flags = CAN_MSG_FLAG_RTR | CAN_MSG_FLAG_EXTD;

	txmsg.data_length_code = len;
	memcpy(&txmsg.data[0], &data[0], len);
	//ESP_LOGI(TAG, "CAN tx id %x len %d", id, len);
	if (can_transmit(&txmsg, pdMS_TO_TICKS(1000)) != ESP_OK)
	{
		ESP_LOGI(TAG, "CAN tx failed");
		//return;

		uint32_t alerts = 0;
		ESP_LOGI(TAG, "CAN reading alerts");
		can_read_alerts(&alerts, pdMS_TO_TICKS(1000));
		ESP_LOGI(tag, "---Alert Read: -- : %04x", alerts);

		if (alerts & CAN_ALERT_RECOVERY_IN_PROGRESS)
		{
			ESP_LOGI(tag, "Recovery in progress");
		}

		if (alerts & CAN_ALERT_ABOVE_ERR_WARN)
		{
			ESP_LOGI(tag, "Surpassed Error Warning Limit");
		}

		if (alerts & CAN_ALERT_ERR_PASS)
		{
			ESP_LOGI(tag, "Entered Error Passive state");
		}

		if (alerts & CAN_ALERT_ERR_ACTIVE)
		{
			ESP_LOGI(tag, "Entered Can Error Active");
		}

		if (alerts & CAN_ALERT_BUS_ERROR)
		{
			ESP_LOGI(tag, "Entered Alert Bus Error");
		}

		if (alerts & CAN_ALERT_BUS_OFF)
		{
			ESP_LOGE(tag, "Bus Off --> Initiate bus recovery");
			can_initiate_recovery(); //Needs 128 occurrences of bus free signal
		}

		//can_clear_transmit_queue();
		//	ESP_LOGE(tag, "--> Initiate bus recovery");
		//can_initiate_recovery();
		//can_start();
		//	ESP_LOGE(tag, " --> Returned from bus recovery");
		vTaskDelay(pdMS_TO_TICKS(1000));

		if (alerts & CAN_ALERT_BUS_RECOVERED)
		{
			//Bus recovery was successful

			// only for testing. Does not help !!
			//esp_err_t res = can_reconfigure_alerts(alerts_enabled, NULL);
			ESP_LOGI(tag, "Bus Recovered"); // %d--> restarting Can", res);

			enable_alerts();

			//put can in start state again and re-enable alert monitoring
			can_start();
		}
	}
	else
	{
		//ESP_LOGI(TAG, "CAN tx success id %x len %d", id, len);
		tx_count++;
	}
	//ESP_LOGI(TAG, "CAN tx end");
}
/*
    #ifdef CAN_DEBUG
                for (int i = 0; i < message.data_length_code; i++)
                {
                    ESP_LOGI(TAG, "byte %02x", message.data[i]);
                }
            }
            ESP_LOGI(TAG, "\n");
#endif

      //vTaskDelay(1000 / portTICK_PERIOD_MS);
    */

void bms_can::can_status_task()
{
	vTaskDelay(pdMS_TO_TICKS(2000));

	for (;;)
	{
		int32_t send_index = 0;
		uint8_t buffer[8];

		// check the health of the bus
		can_status_info_t status;
		can_get_status_info(&status);
		ESP_LOGI(TAG, "  %d/%d, rx-q:%d, tx-q:%d, rx-err:%d, tx-err:%d, arb-lost:%d, bus-err:%d, state: %s",
				 bms_can::rx_count, bms_can::tx_count,
				 status.msgs_to_rx, status.msgs_to_tx, status.rx_error_counter, status.tx_error_counter, status.arb_lost_count,
				 status.bus_error_count, ESP32_CAN_STATUS_STRINGS[status.state]);
		if (status.state == CAN_STATE_STOPPED)
		{
			//Start CAN driver
			if (can_start() == ESP_OK)
			{
				ESP_LOGI(TAG, "CAN Driver started\n");
			}
			else
			{
				ESP_LOGI(TAG, "Failed to start driver\n");
			}
		}

		buffer_append_float32_auto(buffer, bms_if_get_v_tot(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_v_charge(), &send_index);
		can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_V_TOT << 8), buffer, send_index);

		send_index = 0;
		buffer_append_float32_auto(buffer, bms_if_get_i_in(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_i_in_ic(), &send_index);
		can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_I << 8), buffer, send_index);

		send_index = 0;
		buffer_append_float32_auto(buffer, bms_if_get_ah_cnt(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_wh_cnt(), &send_index);
		can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_AH_WH << 8), buffer, send_index);

		int cell_now = 0;
		int cell_max = (settings->totalNumberOfSeriesModules);
		while (cell_now < cell_max)
		{
			send_index = 0;
			buffer[send_index++] = cell_now - 0;
			buffer[send_index++] = settings->totalNumberOfSeriesModules;
			if (cell_now < cell_max)
			{
				buffer_append_float16(buffer, bms_if_get_v_cell(cell_now++), 1e3, &send_index);
			}
			if (cell_now < cell_max)
			{
				buffer_append_float16(buffer, bms_if_get_v_cell(cell_now++), 1e3, &send_index);
			}
			if (cell_now < cell_max)
			{
				buffer_append_float16(buffer, bms_if_get_v_cell(cell_now++), 1e3, &send_index);
			}
			can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_V_CELL << 8), buffer, send_index);
		}

		send_index = 0;
		buffer[send_index++] = settings->totalNumberOfSeriesModules;
		uint64_t bal_state = 0;
		for (int i = 0; i < cell_max; i++)
		{
			bal_state |= (uint64_t)bms_if_is_balancing_cell(i) << i;
		}
		buffer[send_index++] = (bal_state >> 48) & 0xFF;
		buffer[send_index++] = (bal_state >> 40) & 0xFF;
		buffer[send_index++] = (bal_state >> 32) & 0xFF;
		buffer[send_index++] = (bal_state >> 24) & 0xFF;
		buffer[send_index++] = (bal_state >> 16) & 0xFF;
		buffer[send_index++] = (bal_state >> 8) & 0xFF;
		buffer[send_index++] = (bal_state >> 0) & 0xFF;
		can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_BAL << 8), buffer, send_index);

		int temp_now = 0;
		while (temp_now < HW_ADC_TEMP_SENSORS)
		{
			send_index = 0;
			buffer[send_index++] = temp_now;
			buffer[send_index++] = HW_ADC_TEMP_SENSORS;
			if (temp_now < HW_ADC_TEMP_SENSORS)
			{
				buffer_append_float16(buffer, bms_if_get_temp(temp_now++), 1e2, &send_index);
			}
			if (temp_now < HW_ADC_TEMP_SENSORS)
			{
				buffer_append_float16(buffer, bms_if_get_temp(temp_now++), 1e2, &send_index);
			}
			if (temp_now < HW_ADC_TEMP_SENSORS)
			{
				buffer_append_float16(buffer, bms_if_get_temp(temp_now++), 1e2, &send_index);
			}
			can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_TEMPS << 8), buffer, send_index);
		}

		send_index = 0;
		buffer_append_float16(buffer, bms_if_get_humidity_sensor_temp(), 1e2, &send_index);
		buffer_append_float16(buffer, bms_if_get_humitidy(), 1e2, &send_index);
		buffer_append_float16(buffer, bms_if_get_temp_ic(), 1e2, &send_index); // Put IC temp here instead of making mew msg
		can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_HUM << 8), buffer, send_index);

		/*
		 * CAN_PACKET_BMS_SOC_SOH_TEMP_STAT
		 *
		 * b[0] - b[1]: V_CELL_MIN (mV)
		 * b[2] - b[3]: V_CELL_MAX (mV)
		 * b[4]: SoC (0 - 255)
		 * b[5]: SoH (0 - 255)
		 * b[6]: T_CELL_MAX (-128 to +127 degC)
		 * b[7]: State bitfield:
		 * [B7      B6      B5      B4      B3      B2      B1      B0      ]
		 * [RSV     RSV     RSV     RSV     RSV     CHG_OK  IS_BAL  IS_CHG  ]
		 */
		send_index = 0;
		buffer_append_float16(buffer, bms_if_get_v_cell_min(), 1e3, &send_index);
		buffer_append_float16(buffer, bms_if_get_v_cell_max(), 1e3, &send_index);
		buffer[send_index++] = (uint8_t)(bms_if_get_soc() * 255.0);
		buffer[send_index++] = (uint8_t)(bms_if_get_soh() * 255.0);
		buffer[send_index++] = (int8_t)HW_TEMP_CELLS_MAX();
		buffer[send_index++] =
			((bms_if_is_charging() ? 1 : 0) << 0) |
			((bms_if_is_balancing() ? 1 : 0) << 1) |
			((bms_if_is_charge_allowed() ? 1 : 0) << 2);
		can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_SOC_SOH_TEMP_STAT << 8), buffer, send_index);

		// DieBie Emulation
		/*
		send_index = 0;
		buffer_append_float32_auto(buffer, bms_if_get_v_tot(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_v_charge(), &send_index);
		can_transmit_eid(settings->controller_id | ((uint32_t)CAN_PACKET_BMS_V_TOT << 8), buffer, send_index);
		*/

		// TODO: allow config
		int32_t sleep_time = 1000 / 1;
		if (sleep_time == 0)
		{
			sleep_time = 1;
		}

		//chThdSleep(sleep_time);
		//delay(sleep_time);
		vTaskDelay(sleep_time / portTICK_PERIOD_MS);
	}
}

float bms_can::bms_if_get_v_tot()
{
	return pi->voltage;
}

float bms_can::bms_if_get_v_charge()
{
	return 43.2;
}

float bms_can::bms_if_get_i_in()
{
	return pi->averageCurrent;
}

float bms_can::bms_if_get_i_in_ic()
{
	return pi->averageCurrent;
}

float bms_can::bms_if_get_ah_cnt()
{
	return pi->remainingCapacityAh;
}

float bms_can::bms_if_get_wh_cnt()
{
	return 314.1;
}

float bms_can::bms_if_get_v_cell(int cell)
{
	return cmi[cell].voltagemV / 1000.0;
}

// Return range for soc/soh is 0.0 -> 1.0
float bms_can::bms_if_get_soc()
{
	return pi->soc / 100.0;
}

float bms_can::bms_if_get_soh()
{
	return pi->soh / 100.0;
}

bool bms_can::bms_if_is_charging()
{
	return false;
}

bool bms_can::bms_if_is_charge_allowed()
{
	return true;
}

bool bms_can::bms_if_is_balancing()
{
	return true;
}

bool bms_can::bms_if_is_balancing_cell(int cell)
{
	return cmi[cell].inBypass;
}

float bms_can::bms_if_get_v_cell_min()
{
	return pi->minCellVolt;
}

float bms_can::bms_if_get_v_cell_max()
{
	return pi->maxCellVolt;
}

float bms_can::bms_if_get_humidity_sensor_temp()
{
	return 31.4;
}

float bms_can::bms_if_get_humitidy()
{
	return 31.4;
}

float bms_can::bms_if_get_temp_ic()
{
	return pi->temperature_ic1;
}

int bms_can::HW_TEMP_CELLS_MAX()
{
	2;
}

float bms_can::bms_if_get_temp(int sensor)
{
	return pi->temperature1;
}

//uint8_t totalNumberOfBanks;
//  uint8_t totalNumberOfSeriesModules;

int bms_can::rxcnt(void)
{
	return bms_can::rx_count;
}

int bms_can::txcnt(void)
{
	return bms_can::tx_count;
}
