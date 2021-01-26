const i2c = require('i2c-bus');

const busNumber = 1;
const i2cAddress = 0x76;

bus = i2c.openSync(busNumber);

var digT = [];
var digP = [];
var digH = [];

var t_fine = 0.0;

function getCalibParam() {
    let calib = Buffer.alloc(32);

    bus.readI2cBlockSync(i2cAddress, 0x88, 24, calib);
    bus.readI2cBlockSync(i2cAddress, 0xA1, 1, calib.subarray(24));
    bus.readI2cBlockSync(i2cAddress, 0xE1, 7, calib.subarray(25));
    
    digT.push((calib[1] << 8) | calib[0]);
	digT.push((calib[3] << 8) | calib[2]);
	digT.push((calib[5] << 8) | calib[4]);
	digP.push((calib[7] << 8) | calib[6]);
	digP.push((calib[9] << 8) | calib[8]);
	digP.push((calib[11]<< 8) | calib[10]);
	digP.push((calib[13]<< 8) | calib[12]);
	digP.push((calib[15]<< 8) | calib[14]);
	digP.push((calib[17]<< 8) | calib[16]);
	digP.push((calib[19]<< 8) | calib[18]);
	digP.push((calib[21]<< 8) | calib[20]);
	digP.push((calib[23]<< 8) | calib[22]);
	digH.push( calib[24] );
	digH.push((calib[26]<< 8) | calib[25]);
	digH.push( calib[27] );
	digH.push((calib[28]<< 4) | (0x0F & calib[29]));
	digH.push((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F));
    digH.push( calib[31] );
    
    for (let i = 1; i < 2; i++) if (digT[i] & 0x8000) digT[i] = (-digT[i] ^ 0xFFFF) + 1;
    for (let i = 1; i < 8; i++) if (digP[i] & 0x8000) digP[i] = (-digP[i] ^ 0xFFFF) + 1;
    for (let i = 0; i < 6; i++) if (digH[i] & 0x8000) digH[i] = (-digH[i] ^ 0xFFFF) + 1;
}

function readData() {
    let data = Buffer.alloc(8);
    bus.readI2cBlockSync(i2cAddress, 0xF7, 8, data);
	let pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
	let temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
	let hum_raw  = (data[6] << 8)  |  data[7]
    
    return {
        temperature: compensate_T(temp_raw),
        pressure: compensate_P(pres_raw),
        humidity: compensate_H(hum_raw),
    };
}

function compensate_P(adc_P) {
    let pressure = 0.0;

    let v1 = (t_fine / 2.0) - 64000.0;
    let v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5];
    v2 = v2 + ((v1 * digP[4]) * 2.0);
    v2 = (v2 / 4.0) + (digP[3] * 65536.0);
    v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144;
    v1 = ((32768 + v1) * digP[0]) / 32768;

    if (v1 == 0) return 0;
    pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125;
    if (pressure < 0x80000000) pressure = (pressure * 2.0) / v1;
    else pressure = (pressure / v1) * 2;
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096;
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0;

    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0);
    return pressure / 100;
}

function compensate_T(adc_T) {
	let v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1];
	let v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2];
	t_fine = v1 + v2;
	let temperature = t_fine / 5120.0;
    return temperature;
}

function compensate_H(adc_H) {
	let var_h = t_fine - 76800.0;
	if (var_h == 0) return 0;
    var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)));
    
	if (var_h > 100.0) return 100.0;
    else if (var_h < 0.0) return 0.0;
    else return var_h;
}

function setup() {
    let osrs_t = 1;			// Temperature oversampling x 1
	let osrs_p = 1;			// Pressure oversampling x 1
	let osrs_h = 1;			// Humidity oversampling x 1
	let mode   = 3;			// Normal mode
	let t_sb   = 5;			// Tstandby 1000ms
	let filter = 0;			// Filter off
	let spi3w_en = 0;			// 3-wire SPI Disable

	let ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
	let config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
	let ctrl_hum_reg  = osrs_h;

	bus.writeByteSync(i2cAddress, 0xF2 ,ctrl_hum_reg);
	bus.writeByteSync(i2cAddress, 0xF4 ,ctrl_meas_reg);
	bus.writeByteSync(i2cAddress, 0xF5 ,config_reg);
}

setup();
getCalibParam();

if (require.main === module) {
    let data = readData();
    console.log(`temp: ${data.temperature}°C`);
    console.log(`pressure: ${data.pressure} hPa`);
    console.log(`hum: ${data.humidity} %`);
}