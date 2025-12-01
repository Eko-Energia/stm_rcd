
#include "app.h"
#include "type2.h"
#include "can_driver.h"

void app_main()
{
	while(1)
	{
		Type2_Process();

		switch(Type2_state)
		{
		case Type2_DISCONNECTED:
			break;
		case Type2_IDLE:
			// jezeli stan wysoki na transoptorach i styczniki rozwarte
			// safestate
			// jezeli stan wysoki na transoptorach
			// stopCharging()
			break;
		case Type2_CHARGING:
			// jeżeli stan niski na transoptorach i styczniki rozwarte
			// startCharging()
			// jezeli styczniki zwarte i stan niski na transoptorach
			// czekaj
				// jezeli dlug o czekasz to coś jest nie tak
			// jezeli styczniki zwarte i stan wysoki na transoptorach
			// git
			break;
		default:
			// error handler unknown type2_state
			break;
		}
	}
}

static void StartCharging()
{

}


