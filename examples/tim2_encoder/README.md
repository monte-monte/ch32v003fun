# Timer 2 quadrature encoder example
This example use the Timer 2 module in quadrature encoder mode to count up/down as a rotary encoder is moved

## Use
Connect A/B pins of encoder to PC2/PC5.  Spin the encoder and see the count increase/decrease on the debug monitor.

### Note:
 * Port D pins and C0,C4, and C7 have a diode that clamps the pin to Vcc and are not 5V tolerant when Vcc is 3V3
 * Port C pins 1,2,3,5, and 6 are not clamped to Vcc and are should be 5V tolerant.
 * Encoder mode uses CH1/CH2 so Timer2 must be used with a PARTIALREMAP1 so CH1/CH2 pins are on 5V tolerant pins.
 * If you don't require 5V tolerance you can use any of the REMAP options

## Pin Remapping options
<table>
  <tr>
    <th colspan="2">00  --  AFIO_PCFR1_TIM2_REMAP_NOREMAP</th>
  </tr>
  <tr>
    <td>D4</td>
    <td>T2CH1ETR</td>
  </tr>
  <tr>
    <td>D3</td>
    <td>T2CH2</td>
  </tr>
  <tr>
    <td>C0</td>
    <td>T2CH3</td>
  </tr>
  <tr>
    <td>D7</td>
    <td>T2CH4  --note: requires disabling nRST in opt</td>
  </tr>
  <tr>
    <th colspan="2">01  --  AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1</th>
  </tr>
  <tr>
    <td>C5</td>
    <td>T2CH1ETR_</td>
  </tr>
  <tr>
    <td>C2</td>
    <td>T2CH2_</td>
  </tr>
  <tr>
    <td>D2</td>
    <td>T2CH3_</td>
  </tr>
  <tr>
    <td>C1</td>
    <td>T2CH4_</td>
  </tr>
  <tr>
    <th colspan="2">10  --  AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2</th>
  </tr>
  <tr>
    <td>C1</td>
    <td>T2CH1ETR_</td>
  </tr>
  <tr>
    <td>D3</td>
    <td>T2CH2</td>
  </tr>
  <tr>
    <td>C0</td>
    <td>T2CH3</td>
  </tr>
  <tr>
    <td>D7</td>
    <td>T2CH4  --note: requires disabling nRST in opt </td>
  </tr>
  <tr>
    <th colspan="2">11  --  AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1</th>
  </tr>
  <tr>
    <td>C1</td>
    <td>T2CH1ETR_</td>
  </tr>
  <tr>
    <td>C7</td>
    <td>T2CH2_</td>
  </tr>
  <tr>
    <td>D6</td>
    <td>T2CH3_</td>
  </tr>
  <tr>
    <td>D5</td>
    <td>T2CH4_</td>
  </tr>
</table>
