#include <app_event_manager.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/pm/device.h>
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


static K_EVENT_DEFINE(button_events);


int main(void)
{
	const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));
#if defined(CONFIG_APP_SUSPEND_CONSOLE)
	const struct device *cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif
	static const struct adc_dt_spec battery_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
	int ret;
	uint32_t reset_cause;
	int main_wdt_chan_id = -1;
	uint32_t events;
	int16_t buf;
	int32_t val_mv;
	uint32_t network_val_mv;

	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
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


	k_sleep(K_SECONDS(2));

	struct sockaddr_in6 serv_addr;
	int sockfd = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

	serv_addr.sin6_family = AF_INET6;
	serv_addr.sin6_port = htons(MY_PC_PORT);

	ret = inet_pton(AF_INET6, MY_PC_ADDR6, &serv_addr.sin6_addr);
	if (ret <= 0) {
		LOG_ERR("Invalid address / Address not supported");
		return ret;
	}

	ret = connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
	if (ret < 0) {
		LOG_ERR("Connect failed");
		return ret;
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
		       buf);

		val_mv = buf * 5; // Divided by 5 at source (NRF_SAADC_VDDHDIV5)
		ret = adc_raw_to_millivolts_dt(&battery_adc,
					       &val_mv);
		if (ret < 0) {
			LOG_ERR("Value in mV not available");
		} else {
			LOG_INF("🔋 = %"PRId32" mV", val_mv);
		}

		network_val_mv = htonl(val_mv);
		ret = send(sockfd, &network_val_mv, sizeof(network_val_mv), 0);
		if (ret < 0) {
			LOG_ERR("Could not send (%d)", ret);
		}

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
