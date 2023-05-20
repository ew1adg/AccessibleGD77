#include "functions/calibration.h"
#include "functions/settings.h"
#include "user_interface/menuSystem.h"
#include "user_interface/uiUtilities.h"
#include "user_interface/uiLocalisation.h"

static menuStatus_t menuRSSIExitCode = MENU_STATUS_SUCCESS;
static void updateScreen(bool forceRedraw, bool firstRun);
static void handleEvent(uiEvent_t *ev);


menuStatus_t menuFMradio(uiEvent_t *ev, bool isFirstRun)
{
	static uint32_t m = 0;

	if (isFirstRun)
	{
		menuDataGlobal.endIndex = 0;
		ucClearBuf();
		menuDisplayTitle(currentLanguage->fmradio);
		ucRenderRows(0, 2);

		updateScreen(true, true);
	}
	else
	{
		menuRSSIExitCode = MENU_STATUS_SUCCESS;
		if (ev->hasEvent)
		{
			handleEvent(ev);
		}

		if((ev->time - m) > RSSI_UPDATE_COUNTER_RELOAD)
		{
			m = ev->time;
			updateScreen(false, false);
		}
	}

	return menuRSSIExitCode;
}

static void updateScreen(bool forceRedraw, bool isFirstRun)
{
	// Clear whole drawing region
	ucFillRect(0, 14, DISPLAY_SIZE_X, DISPLAY_SIZE_Y - 14, true);

	uint8_t upper_freq = 99;
	uint8_t lower_freq = 10;

	char buffer[16] = {0};
	snprintf(buffer, SCREEN_LINE_BUFFER_SIZE, "%3d.%02d%s", upper_freq, lower_freq, "MHz");
	ucPrintCentered(DISPLAY_Y_POS_RSSI_VALUE, buffer, FONT_SIZE_3);

	ucRender();
}

static void handleEvent(uiEvent_t *ev)
{
	if (KEYCHECK_SHORTUP(ev->keys, KEY_GREEN) || KEYCHECK_SHORTUP(ev->keys, KEY_RED))
	{
		menuSystemPopPreviousMenu();
		return;
	}
	else if (KEYCHECK_SHORTUP_NUMBER(ev->keys)  && (BUTTONCHECK_DOWN(ev, BUTTON_SK2)))
	{
		saveQuickkeyMenuIndex(ev->keys.key, menuSystemGetCurrentMenuNumber(), 0, 0);
		return;
	}
}
