# Port for STM32F446RE

The code by @leech001/MPU6050 has been ported to work with F446RE. 

```Cause for this port - I thought it would be pretty much copying the src and include files
    to the required project. But as i did so, I faced a bit of issue, since the HAL architecture is 
    very different for Cortex-M0 vs Cortex-M4 cores. I also faced issue while calling the device by
    its address 0x68. I have called it by left shifting 0x68 by 1 (0x68 << 1). I thought this will be 
    very handy if someone wants to try it straight fwd into F446RE
