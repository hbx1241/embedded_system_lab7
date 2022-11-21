# Embedded System Lab7
## About
這次的實驗是關於利用arm所提供的api實作一個DSP program，而我們採用的是fir low pass filter來對輸入資料濾波
## How to Start
### Step 1
在mbed studio建立一個空白的project，然後把資料夾`BSP_L475E-IOT01`(Lab2使用過的)以及這個repo的`main.cpp`複製到project裡面。
如果要已知的input來驗證fir low pass filter的正確性，可以使用`check_correctness`內的`arm_fir_data.c`以及`main.cpp`來確認。
### Step 2
開始編譯程式，編譯完後灌入stm32開發板中即可開始執行
### Step 3
程式結束執行以後(預設收集約16秒的sensor資料)，可以在mbed studio的terminal上觀察到input和output的數值。透過excel等工具可以把資料畫出來觀察
## Demo
