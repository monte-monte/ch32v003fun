# Timer 2 AFIO demo
This example use the AFIO (Alternate Function Input Output) until to remap the outputs of Timer 2

## Use
Connect LEDs between PD3 and GND, PD4 and GND, PC1 and GND, and PC7 and GND. Observe activity on PD3 and PD4, then activity on PC1 and PC7, and back.

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
