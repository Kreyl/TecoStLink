// Initialization @ GD32E103
    typedef enum {
      SWO_Manchester = 0x1,
      SWO_Async = 0x2,
    } swoProtocolType;

    swoProtocolType swoProtocol = SWO_Manchester;
    uint32_t swoSpeedBaud = 115200;
    if (swoProtocol == SWO_Manchester) swoSpeedBaud = 2 * swoSpeedBaud;
    uint32_t swoPrescaler = (Clk::AHBFreqHz / swoSpeedBaud) - 1;

    CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk; /* trace enable */
    TPI->CSPSR = 1;                                /* protocol width = 1 bit */
    TPI->SPPR = swoProtocol;  /* 1 = Manchester, 2 = Asynchronous */
    TPI->ACPR = swoPrescaler; /* frequency */
    TPI->FFCR = 0;            /* turn off formatter, discard ETM output */
    //TPI->FFCR = 0x102; // Default value            /* turn off formatter, discard ETM output */

    ITM->LAR = 0xC5ACCE55;    /* unlock access to ITM registers */
    ITM->TCR = ITM_TCR_SWOENA_Msk | ITM_TCR_ITMENA_Msk; /* trace control register */
    ITM->TPR = 0; /* all ports accessible unprivileged */
    ITM->TER = 0xFFFFFFFF; /* enable all stimulus channels */

    DBGMCU->CTL |= 1UL << 5; // En TRACE_IO
	
// Sending chars
                ITM_SendChar('a');
                ITM_SendChar('g');
                ITM_SendChar('a');
                ITM_SendChar('\r');