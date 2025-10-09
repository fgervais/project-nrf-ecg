#include <app_event_manager.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/net/coap_client.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/pm/device.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/debug/thread_analyzer.h>

#define MODULE main
#include <caf/events/module_state_event.h>
#include <caf/events/button_event.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include <app_version.h>
#include <mymodule/base/openthread.h>
#include <mymodule/base/reset.h>
#include <mymodule/base/watchdog.h>


#define BUTTON_PRESS_EVENT	BIT(0)

#define MY_PC_ADDR6		"fd04:2240::1cef"
#define MY_PC_PORT		50000

#define COAP_PORT		5683
#define COAP_PATH_ECG		"ecg"

#define DIRECT_GLOBAL_IP6_ADDRESS \
        { { { 0xfd, 0x04, 0x22, 0x40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1c, 0xef } } }

static K_EVENT_DEFINE(button_events);

SENSOR_DT_READ_IODEV(ecg_iodev, DT_NODELABEL(max30001),
		{SENSOR_CHAN_VOLTAGE, 0});

RTIO_DEFINE(ecg_rtio_ctx, 1, 1);


static void on_coap_response(int16_t result_code, size_t offset,
			     const uint8_t *payload, size_t len,
			     bool last_block, void *user_data)
{
	// int *sockfd = (int *)user_data;

	LOG_INF("CoAP response, result_code=%d, offset=%u, len=%u, last_block=%d",
		result_code, offset, len, last_block);

	if (result_code == COAP_RESPONSE_CODE_CHANGED) {
		LOG_INF("🎉 CoAP succeeded");
	}
	else {
		LOG_ERR("Error during CoAP download, result_code=%d", result_code);
	}

	// openthread_request_normal_latency("coap response");

	// zsock_close(*sockfd);
}

static int send_ecg_buffer(struct coap_client *client,
			   int sockfd,
			   struct sockaddr *sa,
			   uint32_t *buf,
			   size_t len)
{
	int ret;
	// int sockfd;
	struct coap_client_request request = {
		.method = COAP_METHOD_POST,
		.confirmable = true,
		.path = COAP_PATH_ECG,
		.fmt = COAP_CONTENT_FORMAT_APP_OCTET_STREAM,
		.payload = (uint8_t *)buf,
		.len = len * sizeof(uint32_t),
		.cb = on_coap_response,
		.options = NULL,
		.num_options = 0,
		// .user_data = &sockfd,
		.user_data = NULL,
	};

	// sockfd = zsock_socket(sa->sa_family, SOCK_DGRAM, 0);
	// if (sockfd < 0) {
	// 	LOG_ERR("Failed to create socket, err %d", errno);
	// 	return -errno;
	// }

	LOG_INF("Starting CoAP request");

	// openthread_request_low_latency("coap request");

	ret = coap_client_req(client, sockfd, sa, &request, NULL);
	if (ret) {
		LOG_ERR("Failed to send CoAP request, err %d", ret);
		// openthread_request_normal_latency("coap request error");
		return ret;
	}

	return 0;
}

int main(void)
{
	const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));
#if defined(CONFIG_APP_SUSPEND_CONSOLE)
	const struct device *cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif
	static const struct adc_dt_spec battery_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
	const struct device *ecg = DEVICE_DT_GET(DT_NODELABEL(max30001));
	int ret;
	uint32_t reset_cause;
	int main_wdt_chan_id = -1;
	uint32_t events;
	int16_t adc_buf;
	int32_t val_mv;
	uint32_t network_val_mv;
	uint8_t ecg_buf[32] = {0};
	struct sensor_value value_ecg = {0};
	int i;

	struct adc_sequence sequence = {
		.buffer = &adc_buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(adc_buf),
	};

	ret = watchdog_new_channel(wdt, &main_wdt_chan_id);
	if (ret < 0) {
		LOG_ERR("Could allocate main watchdog channel");
		return ret;
	}

	ret = watchdog_start(wdt);
	if (ret < 0) {
		LOG_ERR("Could allocate start watchdog");
		return ret;
	}

	LOG_INF("\n\n🚀 MAIN START (%s) 🚀\n", APP_VERSION_FULL);

	reset_cause = show_and_clear_reset_cause();
	
	if (app_event_manager_init()) {
		LOG_ERR("Event manager not initialized");
	} else {
		module_set_state(MODULE_STATE_READY);
	}

	if (!device_is_ready(battery_adc.dev)) {
		LOG_ERR("ADC controller device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(ecg)) {
		LOG_ERR("Device \"%s\" is not ready",
		       ecg->name);
		return -ENODEV;
	}

	ret = adc_channel_setup_dt(&battery_adc);
	if (ret < 0) {
		LOG_ERR("Could not setup battery ADC (%d)", ret);
		return ret;
	}



	ret = openthread_my_start();
	if (ret < 0) {
		LOG_ERR("Could not start openthread");
		return ret;
	}

	LOG_INF("💤 waiting for openthread to be ready");
	openthread_wait(OT_ROLE_SET | 
			OT_ROUTABLE_ADDR_SET | 
			OT_HAS_NEIGHBORS);



	LOG_INF("🆗 initialized");

#if defined(CONFIG_APP_SUSPEND_CONSOLE)
	ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
	if (ret < 0) {
		LOG_ERR("Could not suspend the console");
		return ret;
	}
#endif

	thread_analyzer_print(0);


	// struct sockaddr_in6 *broker6 = (struct sockaddr_in6 *)&broker;

	// broker6->sin6_family = AF_INET6;
	// broker6->sin6_port = htons(CONFIG_MY_MODULE_BASE_HA_MQTT_SERVER_PORT);
	// zsock_inet_pton(AF_INET6, CONFIG_MY_MODULE_BASE_HA_MQTT_SERVER_ADDR, &broker6->sin_addr);


	k_sleep(K_MSEC(50));

	// struct sockaddr_in6 serv_addr;
	// int sockfd = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

	// serv_addr.sin6_family = AF_INET6;
	// // serv_addr.sin6_port = htons(MY_PC_PORT);
	// serv_addr.sin6_port = htons(COAP_PORT);

	// ret = inet_pton(AF_INET6, MY_PC_ADDR6, &serv_addr.sin6_addr);
	// if (ret <= 0) {
	// 	LOG_ERR("Invalid address / Address not supported");
	// 	return ret;
	// }

	// ret = connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
	// if (ret < 0) {
	// 	LOG_ERR("Connect failed");
	// 	return ret;
	// }





	static struct coap_client coap_client;
	struct sockaddr_in6 sockaddr6 = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(COAP_PORT),
		// .sin6_addr = DIRECT_GLOBAL_IP6_ADDRESS,
	};
	int sockfd;

	ret = inet_pton(AF_INET6, MY_PC_ADDR6, &sockaddr6.sin6_addr);
	if (ret <= 0) {
		LOG_ERR("Invalid address / Address not supported");
		return ret;
	}

	ret = coap_client_init(&coap_client, NULL);
	if (ret) {
		LOG_ERR("Failed to init coap client, err %d", ret);
		return ret;
	}

	sockfd = zsock_socket(sockaddr6.sin6_family, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		LOG_ERR("Failed to create socket, err %d", errno);
		return -errno;
	}

	// ret = zsock_setsockopt(sockfd,
	// 		       IPPROTO_IPV6,
	// 		       IPV6_MULTICAST_HOPS,
	// 		       &mcast_hops,
        //                        sizeof(mcast_hops));




	// for (i = 0; i<10; i++) {
	// 	ret = sensor_sample_fetch(ecg);
	// 	if (ret) {
	// 		printk("sensor_sample_fetch failed ret %d\n", ret);
	// 		return 0;
	// 	}

	// 	ret = sensor_channel_get(ecg, SENSOR_CHAN_VOLTAGE, &value_ecg);
	// 	LOG_INF("🫀 %f", sensor_value_to_double(&value_ecg));

	// 	// k_sleep(K_MSEC(8));
	// }


	struct sensor_q31_data ecg_data = {0};
	struct sensor_decode_context ecg_decoder = SENSOR_DECODE_CONTEXT_INIT(
		SENSOR_DECODER_DT_GET(DT_NODELABEL(max30001)),
		ecg_buf, SENSOR_CHAN_VOLTAGE, 0);
	uint32_t network_ecg_voltage;
	uint32_t network_ecg_buffer[32];
	size_t buffer_len = 0;


	for (i = 0; i<100; i++) {
		ret = sensor_read(&ecg_iodev, &ecg_rtio_ctx,
				  ecg_buf, sizeof(ecg_buf));
		if (ret != 0) {
			LOG_ERR("%s: sensor_read() failed: %d\n", ecg->name, ret);
			return ret;
		}

		// LOG_INF("🫀 %d", events);
		// LOG_HEXDUMP_INF(ecg_buf, sizeof(ecg_buf), "");

		ret = sensor_decode(&ecg_decoder, &ecg_data, 1);
		if (ret == -ENODATA) {
			send_ecg_buffer(&coap_client,
					sockfd,
					(struct sockaddr *)&sockaddr6,
					// (struct sockaddr *)&serv_addr,
					network_ecg_buffer,
					buffer_len);

			buffer_len = 0;

			k_msleep(50);
			continue;
		}
		if (ret < 0) {
			LOG_ERR("%s: sensor_decode() failed: %d\n",
				ecg->name, ret);
			break;
		}

		LOG_INF("🫀 Decoded ECG %" PRIsensor_q31_data,
		       PRIsensor_q31_data_arg(ecg_data, 0));

		network_ecg_buffer[buffer_len] = htonl(ecg_data.readings[0].voltage);
		buffer_len += 1;

		// network_ecg_voltage = htonl(ecg_data.readings[0].voltage);
		// ret = send(sockfd, &network_ecg_voltage,
		// 	   sizeof(network_ecg_voltage), 0);
		// if (ret < 0) {
		// 	LOG_ERR("Could not send (%d)", ret);
		// }

		ecg_decoder.fit = 0;
		wdt_feed(wdt, main_wdt_chan_id);
		k_msleep(1);
	}




	LOG_INF("┌──────────────────────────────────────────────────────────┐");
	LOG_INF("│ Entering main loop                                       │");
	LOG_INF("└──────────────────────────────────────────────────────────┘");

	while (1) {
		LOG_INF("💤 waiting for events");
		events = k_event_wait(&button_events,
				(BUTTON_PRESS_EVENT),
				true,
				K_SECONDS(CONFIG_APP_MAIN_LOOP_PERIOD_SEC));

		LOG_INF("⏰ events: %08x", events);

		if (events & BUTTON_PRESS_EVENT) {
			LOG_INF("handling button press event");
		}


		LOG_INF("ADC reading:");

		adc_sequence_init_dt(&battery_adc, &sequence);
		ret = adc_read(battery_adc.dev, &sequence);
		if (ret < 0) {
			LOG_ERR("Could not read (%d)", ret);
			continue;
		}

		LOG_INF("%s, channel %d: %d",
		       battery_adc.dev->name,
		       battery_adc.channel_id,
		       adc_buf);

		val_mv = adc_buf * 5; // Divided by 5 at source (NRF_SAADC_VDDHDIV5)
		ret = adc_raw_to_millivolts_dt(&battery_adc,
					       &val_mv);
		if (ret < 0) {
			LOG_ERR("Value in mV not available");
		} else {
			LOG_INF("🔋 = %"PRId32" mV", val_mv);
		}

		// network_val_mv = htonl(val_mv);
		// ret = send(sockfd, &network_val_mv, sizeof(network_val_mv), 0);
		// if (ret < 0) {
		// 	LOG_ERR("Could not send (%d)", ret);
		// }

		LOG_INF("🦴 feed watchdog");
		wdt_feed(wdt, main_wdt_chan_id);
	}

	return 0;
}

static bool event_handler(const struct app_event_header *eh)
{
	const struct button_event *evt;

	if (is_button_event(eh)) {
		evt = cast_button_event(eh);

		if (evt->pressed) {
			LOG_INF("🛎️  Button pressed");
			k_event_post(&button_events, BUTTON_PRESS_EVENT);
		}
	}

	return true;
}

APP_EVENT_LISTENER(MODULE, event_handler);
APP_EVENT_SUBSCRIBE(MODULE, button_event);
