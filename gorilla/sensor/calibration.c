/*
 * File      : calibration.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-30     weety    first version.
 */

#include "global.h"
#include "calibration.h"
#include "param.h"
#include "sensor.h"
#include <math.h>
#include <finsh.h>
#include <shell.h>

extern sensor_t sensor_orig_acc;
extern sensor_t sensor_orig_gyr;

void cali_obj_init(Cali_Obj *obj)
{
	for(int i = 0 ; i < 9 ; i++){
		obj->V[i] = 0.0f;
		obj->D[i] = 0.0f;
		for(int j = 0 ; j < 9 ; j++){
			obj->P[i][j] = 0.0f;
		}
	}
	
	obj->P[0][0] = obj->P[1][1] = obj->P[2][2] = 10.0f;
	obj->P[3][3] = obj->P[4][4] = obj->P[5][5] = 1.0f;
	obj->P[6][6] = obj->P[7][7] = obj->P[8][8] = 1.0f;
	obj->R = 0.001f;
	
	MatCreate(&obj->EigVec, 3, 3);
	MatCreate(&obj->RotM, 3, 3);
}

void cali_obj_delete(Cali_Obj *obj)
{
	MatDelete(&obj->EigVec);
	MatDelete(&obj->RotM);
}

void cali_least_squre_update(Cali_Obj *obj, float val[3])
{
	double x = val[0];
	double y = val[1];
	double z = val[2];
	
	obj->D[0] = x*x;
	obj->D[1] = y*y;
	obj->D[2] = z*z;
	obj->D[3] = 2.0f*x*y;
	obj->D[4] = 2.0f*x*z;
	obj->D[5] = 2.0f*y*z;
	obj->D[6] = 2.0f*x;
	obj->D[7] = 2.0f*y;
	obj->D[8] = 2.0f*z;
	
	// Y = Z-D*V
	float DV = 0.0f;
	for(uint8_t i = 0 ; i < 9 ; i++){
		DV += obj->D[i]*obj->V[i];
	}
	float Y = 1.0f - DV;
	
	// S = D*P*D' + R
	double DP[9];
	for(uint8_t i = 0 ; i < 9 ; i++){
		DP[i] = 0.0f;
		for(uint8_t j = 0 ; j < 9 ; j++){
			DP[i] += obj->D[j]*obj->P[j][i];
		}
	}
	double DPDT = 0.0f;
	for(uint8_t i = 0 ; i < 9 ; i++){
		DPDT += DP[i] * obj->D[i];
	}
	double S = DPDT + obj->R;
	
	// K = P*D'/S
	double K[9];
	for(uint8_t i = 0 ; i < 9 ; i++){
		K[i] = 0.0f;
		for(uint8_t j = 0 ; j < 9 ; j++){
			K[i] += obj->P[i][j] * obj->D[j];
		}
		K[i] = K[i]/S;
	}
	
	// V = V + K*Y
	for(uint8_t i = 0 ; i < 9 ; i++){
		obj->V[i] += K[i]*Y;
	}
	
	// P = P - K*D*P
	double KD[9][9];
	for(uint8_t i = 0 ; i < 9 ; i++){
		for(uint8_t j = 0 ; j < 9 ; j++){
			KD[i][j] = K[i] * obj->D[j];
		}
	}
	double KDP[9][9];
	for(uint8_t i = 0 ; i < 9 ; i++){
		for(uint8_t j = 0 ; j < 9 ; j++){
			KDP[i][j] = 0.0f;
			for(uint8_t k = 0 ; k < 9 ; k++){
				KDP[i][j] += KD[i][k] * obj->P[j][k];
			}
		}
	}	
	for(uint8_t i = 0 ; i < 9 ; i++){
		for(uint8_t j = 0 ; j < 9 ; j++){
			obj->P[i][j] -= KDP[i][j];
		}
	}	
}

void cali_solve(Cali_Obj *obj, double radius)
{
	Mat A, B;
	Mat InvB;
	Mat Tmtx;
	Mat AT;
	Mat TmtxA, TmtxTrans;
	Mat E;
	Mat GMat;
	Mat InvEigVec;
	
	MatCreate(&A, 4, 4);
	MatCreate(&B, 3, 3);
	MatCreate(&InvB, 3, 3);
	MatCreate(&Tmtx, 4, 4);
	MatCreate(&AT, 4, 4);
	MatCreate(&TmtxA, 4, 4);
	MatCreate(&TmtxTrans, 4, 4);
	MatCreate(&E, 3, 3);
	MatCreate(&GMat, 3, 3);
	MatCreate(&InvEigVec, 3, 3);
	
	LIGHT_MATRIX_TYPE valA[16] = {
		obj->V[0], obj->V[3], obj->V[4], obj->V[6],
		obj->V[3], obj->V[1], obj->V[5], obj->V[7],
		obj->V[4], obj->V[5], obj->V[2], obj->V[8],
		obj->V[6], obj->V[7], obj->V[8],    -1
	};
	MatSetVal(&A, valA);
	
	LIGHT_MATRIX_TYPE valB[9] = {
		obj->V[0], obj->V[3], obj->V[4],
		obj->V[3], obj->V[1], obj->V[5],
		obj->V[4], obj->V[5], obj->V[2],
	};
	MatSetVal(&B, valB);
	
	MatInv(&B, &InvB);
	
	LIGHT_MATRIX_TYPE v1[3] = {obj->V[6], obj->V[7], obj->V[8]};
	for(uint8_t i = 0 ; i < 3 ; i++){
		obj->OFS[i] = 0.0f;
		for(uint8_t j = 0 ; j < 3 ; j++){
			obj->OFS[i] += InvB.element[i][j]*v1[j];
		}
		obj->OFS[i] = -obj->OFS[i];
	}
	
	MatEye(&Tmtx);
	Tmtx.element[3][0] = obj->OFS[0];
	Tmtx.element[3][1] = obj->OFS[1];
	Tmtx.element[3][2] = obj->OFS[2];
	
	MatMul(&Tmtx, &A, &TmtxA);
	MatTrans(&Tmtx, &TmtxTrans);
	MatMul(&TmtxA, &TmtxTrans, &AT);
	
	LIGHT_MATRIX_TYPE valE[9] = {
		-AT.element[0][0]/AT.element[3][3], -AT.element[0][1]/AT.element[3][3], -AT.element[0][2]/AT.element[3][3],
		-AT.element[1][0]/AT.element[3][3], -AT.element[1][1]/AT.element[3][3], -AT.element[1][2]/AT.element[3][3],
		-AT.element[2][0]/AT.element[3][3], -AT.element[2][1]/AT.element[3][3], -AT.element[2][2]/AT.element[3][3]
	};
	MatSetVal(&E, valE);
	
	LIGHT_MATRIX_TYPE eig_val[3];
	MatEig(&E, eig_val, &obj->EigVec, 1e-6, 100);
	
	for(uint8_t i = 0 ; i < 3 ; i++){
		obj->GAIN[i] = sqrt(1.0/eig_val[i]);
	}
	
	/* calculate transform matrix */
	MatZeros(&GMat);
	GMat.element[0][0] = 1.0/obj->GAIN[0]*radius;
	GMat.element[1][1] = 1.0/obj->GAIN[1]*radius;
	GMat.element[2][2] = 1.0/obj->GAIN[2]*radius;
	
	MatInv(&obj->EigVec, &InvEigVec);
	
	Mat tmp;
	MatCreate(&tmp, 3, 3);
	MatMul(&obj->EigVec, &GMat, &tmp);
	MatMul(&tmp, &InvEigVec, &obj->RotM);
	
	MatDelete(&A);
	MatDelete(&B);
	MatDelete(&InvB);
	MatDelete(&Tmtx);
	MatDelete(&AT);
	MatDelete(&TmtxA);
	MatDelete(&TmtxTrans);
	MatDelete(&E);
	MatDelete(&GMat);
	MatDelete(&InvEigVec);
	MatDelete(&tmp);
}

typedef struct {
	rt_device_t device;
	struct rt_semaphore sem;
	void *rx_indicate;
	rt_uint8_t flag;
} cons_dev_t;

static cons_dev_t cons_dev;

rt_err_t cons_dev_rx_ind(rt_device_t dev, rt_size_t size)
{
	/* release semaphore */
	rt_sem_release(&cons_dev.sem);

	return RT_EOK;
}

void cons_dev_init(void)
{
#ifndef RT_USING_POSIX

	rt_device_t device;
	extern const char* finsh_get_device(void);
	const char* device_name = finsh_get_device();

	device = rt_device_find(device_name);
	if( device == RT_NULL )
	{
		rt_kprintf("%s not find\r\n",device_name);
	}
	cons_dev.device = device;

	cons_dev.flag = RT_DEVICE_FLAG_STREAM;
	cons_dev.device->flag &= (~cons_dev.flag);
	rt_sem_init(&(cons_dev.sem), "csem", 0, 0);
	/* save old rx_indicate	*/
	cons_dev.rx_indicate = cons_dev.device->rx_indicate;
	/* set new rx_indicate */
	rt_device_set_rx_indicate(cons_dev.device, cons_dev_rx_ind);
#endif
}

void cons_dev_deinit(void)
{
#ifndef RT_USING_POSIX
	cons_dev.device->flag |= cons_dev.flag;
	/* recovery old rx_indicate	*/
	rt_device_set_rx_indicate(cons_dev.device, cons_dev.rx_indicate);
	/* finsh>> */
	rt_kprintf(FINSH_PROMPT);
#endif
}

char shell_wait_ch(void)
{
#ifdef RT_USING_POSIX
	return getchar();
#else
	char ch[64];
	
	if (rt_sem_take(&cons_dev.sem, RT_WAITING_FOREVER) != RT_EOK) return -1;
	
	rt_device_read(cons_dev.device, 0, &ch, 64);
	
	return ch[0];
#endif
}

int do_acc_calibrate(void)
{
	sensor_acc_t acc;
	float acc_f[3];
	printf("Start to calibrate acc\n");
	
	Cali_Obj obj;
	char ch;
	cali_obj_init(&obj);

	cons_dev_init();

	printf("towards Z-axis to DOWN side, and keep it static...{Y/N}\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y'){
		printf("reading data...\n");
		
		for(int i = 0 ; i < 100 ; i ++){
			mpdc_pull_data(sensor_orig_acc.mpdc, &acc);
			acc_f[0] = acc.x;
			acc_f[1] = acc.y;
			acc_f[2] = acc.z;
			cali_least_squre_update(&obj, acc_f);
			//printf("%lf %lf %lf\n", acc_f[0], acc_f[1], acc_f[2]);
			rt_thread_mdelay(20);
		}
	}else{
		goto finish;
	}
	
	printf("towards Z-axis to UP side, and keep it static...{Y/N}\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y'){
		printf("reading data...\n");
		
		for(int i = 0 ; i < 100 ; i ++){
			mpdc_pull_data(sensor_orig_acc.mpdc, &acc);
			acc_f[0] = acc.x;
			acc_f[1] = acc.y;
			acc_f[2] = acc.z;
			cali_least_squre_update(&obj, acc_f);
			//printf("%lf %lf %lf\n", acc_f[0], acc_f[1], acc_f[2]);
			rt_thread_mdelay(20);
		}
	}else{
		goto finish;
	}
	
	printf("towards X-axis to DOWN side, and keep it static...{Y/N}\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y'){
		printf("reading data...\n");
		
		for(int i = 0 ; i < 100 ; i ++){
			mpdc_pull_data(sensor_orig_acc.mpdc, &acc);
			acc_f[0] = acc.x;
			acc_f[1] = acc.y;
			acc_f[2] = acc.z;
			cali_least_squre_update(&obj, acc_f);
			//printf("%lf %lf %lf\n", acc_f[0], acc_f[1], acc_f[2]);
			rt_thread_mdelay(20);
		}
	}else{
		goto finish;
	}
	
	printf("towards X-axis to UP side, and keep it static...{Y/N}\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y'){
		printf("reading data...\n");
		
		for(int i = 0 ; i < 100 ; i ++){
			mpdc_pull_data(sensor_orig_acc.mpdc, &acc);
			acc_f[0] = acc.x;
			acc_f[1] = acc.y;
			acc_f[2] = acc.z;
			cali_least_squre_update(&obj, acc_f);
			//printf("%lf %lf %lf\n", acc_f[0], acc_f[1], acc_f[2]);
			rt_thread_mdelay(20);
		}
	}else{
		goto finish;
	}
	
	printf("towards Y-axis to DOWN side, and keep it static...{Y/N}\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y'){
		printf("reading data...\n");
		
		for(int i = 0 ; i < 100 ; i ++){
			mpdc_pull_data(sensor_orig_acc.mpdc, &acc);
			acc_f[0] = acc.x;
			acc_f[1] = acc.y;
			acc_f[2] = acc.z;
			cali_least_squre_update(&obj, acc_f);
			//printf("%lf %lf %lf\n", acc_f[0], acc_f[1], acc_f[2]);
			rt_thread_mdelay(20);
		}
	}else{
		goto finish;
	}
	
	printf("towards Y-axis to UP side, and keep it static...{Y/N}\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y'){
		printf("reading data...\n");
		
		for(int i = 0 ; i < 100 ; i ++){
			mpdc_pull_data(sensor_orig_acc.mpdc, &acc);
			acc_f[0] = acc.x;
			acc_f[1] = acc.y;
			acc_f[2] = acc.z;
			cali_least_squre_update(&obj, acc_f);
			//printf("%lf %lf %lf\n", acc_f[0], acc_f[1], acc_f[2]);
			rt_thread_mdelay(20);
		}
	}else{
		goto finish;
	}
	
	cali_solve(&obj, GRAVITY_ACC);
	printf("Center:%f %f %f\n", obj.OFS[0],obj.OFS[1],obj.OFS[2]);
	printf("Radius:%f %f %f\n", obj.GAIN[0],obj.GAIN[1],obj.GAIN[2]);
	printf("Rotation Matrix:\n");
	for(int row = 0 ; row < obj.RotM.row ; row++){
		for(int col = 0 ; col < obj.RotM.col ; col++){
			printf("%.4f\t", obj.RotM.element[row][col]);
		}
		printf("\n");
	}
	
	printf("store to parameter? (Y/N)\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y'){
		param_set_float("ACC_X_OFFSET", obj.OFS[0]);
		param_set_float("ACC_Y_OFFSET", obj.OFS[1]);
		param_set_float("ACC_Z_OFFSET", obj.OFS[2]);
		param_set_float("ACC_TRANS_MAT00", obj.RotM.element[0][0]);
		param_set_float("ACC_TRANS_MAT01", obj.RotM.element[0][1]);
		param_set_float("ACC_TRANS_MAT02", obj.RotM.element[0][2]);
		param_set_float("ACC_TRANS_MAT10", obj.RotM.element[1][0]);
		param_set_float("ACC_TRANS_MAT11", obj.RotM.element[1][1]);
		param_set_float("ACC_TRANS_MAT12", obj.RotM.element[1][2]);
		param_set_float("ACC_TRANS_MAT20", obj.RotM.element[2][0]);
		param_set_float("ACC_TRANS_MAT21", obj.RotM.element[2][1]);
		param_set_float("ACC_TRANS_MAT22", obj.RotM.element[2][2]);
		param_set_int32("ACC_CALIB", 1);
	}
	
finish:
	cons_dev_deinit();
	cali_obj_delete(&obj);
	return 0;
}

int do_acc_calibrate_multipoint(uint32_t point)
{
	sensor_acc_t acc;
	float acc_f[3];
	printf("Start to calibrate acc\n");

	Cali_Obj obj;
	char ch;
	cali_obj_init(&obj);

	cons_dev_init();

	for (int n = 0; n < point; n++) {
		printf("For %d point...{Y/N}\n", n+1);
		ch = shell_wait_ch();
		if (ch == 'Y' || ch == 'y') {
			printf("reading data...\n");

			for (int i = 0; i < 100; i ++) {
				mpdc_pull_data(sensor_orig_acc.mpdc, &acc);
				acc_f[0] = acc.x;
				acc_f[1] = acc.y;
				acc_f[2] = acc.z;
				cali_least_squre_update(&obj, acc_f);
				//printf("%lf %lf %lf\n", acc_f[0], acc_f[1], acc_f[2]);
				rt_thread_delay(20);
			}
		} else {
			goto finish;
		}
	}

	cali_solve(&obj, GRAVITY_ACC);
	printf("Center:%f %f %f\n", obj.OFS[0],obj.OFS[1],obj.OFS[2]);
	printf("Radius:%f %f %f\n", obj.GAIN[0],obj.GAIN[1],obj.GAIN[2]);
	printf("Rotation Matrix:\n");
	for (int row = 0; row < obj.RotM.row; row++) {
		for (int col = 0; col < obj.RotM.col; col++) {
			printf("%.4f\t", obj.RotM.element[row][col]);
		}
		printf("\n");
	}

	printf("store to parameter? (Y/N)\n");
	ch = shell_wait_ch();
	if (ch == 'Y' || ch == 'y') {
		param_set_float("ACC_X_OFFSET", obj.OFS[0]);
		param_set_float("ACC_Y_OFFSET", obj.OFS[1]);
		param_set_float("ACC_Z_OFFSET", obj.OFS[2]);
		param_set_float("ACC_TRANS_MAT00", obj.RotM.element[0][0]);
		param_set_float("ACC_TRANS_MAT01", obj.RotM.element[0][1]);
		param_set_float("ACC_TRANS_MAT02", obj.RotM.element[0][2]);
		param_set_float("ACC_TRANS_MAT10", obj.RotM.element[1][0]);
		param_set_float("ACC_TRANS_MAT11", obj.RotM.element[1][1]);
		param_set_float("ACC_TRANS_MAT12", obj.RotM.element[1][2]);
		param_set_float("ACC_TRANS_MAT20", obj.RotM.element[2][0]);
		param_set_float("ACC_TRANS_MAT21", obj.RotM.element[2][1]);
		param_set_float("ACC_TRANS_MAT22", obj.RotM.element[2][2]);
		param_set_int32("ACC_CALIB", 1);
	}

finish:
	cons_dev_deinit();
	cali_obj_delete(&obj);
	return 0;
}

int do_gyr_calibrate(void)
{
	sensor_gyr_t gyr;
	double sum_gyr[3] = {0.0f,0.0f,0.0f};
	float offset_gyr[3];
	char ch;
	int p_num = 1000;
	
	printf("start to calibrate gyr\n");
	printf("keep the board static...(Y/N)\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y') {
		printf("reading data...\n");
		for(uint32_t i = 0 ; i < p_num ; i++) {
			mpdc_pull_data(sensor_orig_gyr.mpdc, &gyr);
			sum_gyr[0] += gyr.x;
			sum_gyr[1] += gyr.y;
			sum_gyr[2] += gyr.z;
			rt_thread_mdelay(20);
		}
	} else {
		return 1;
	}
	
	offset_gyr[0] = -sum_gyr[0]/p_num;
	offset_gyr[1] = -sum_gyr[1]/p_num;
	offset_gyr[2] = -sum_gyr[2]/p_num;
	
	printf("gyr offset:%f %f %f\r\n\n" , offset_gyr[0],offset_gyr[1],offset_gyr[2]);
	
	printf("store to parameter? (Y/N)\n");
	ch = shell_wait_ch();
	if(ch == 'Y' || ch == 'y') {
		param_set_float("GYR_X_OFFSET", offset_gyr[0]);
		param_set_float("GYR_Y_OFFSET", offset_gyr[1]);
		param_set_float("GYR_Z_OFFSET", offset_gyr[2]);
		param_set_int32("GYR_CALIB", 1);
	}
	return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

int cmd_calibrate(int argc, char** argv)
{
	int res = 0;

	if (argc > 1) {
		if(strcmp("gyr", argv[1]) == 0) {
			res = do_gyr_calibrate();
		}
		
		if(strcmp("acc_old", argv[1]) == 0) {
			res = do_acc_calibrate();
		}

		if (strcmp("acc", argv[1]) == 0) {
			res = do_acc_calibrate_multipoint(14);
		}
	}

	return res;
}

MSH_CMD_EXPORT_ALIAS(cmd_calibrate, calibrate, calibrate sensor);

#endif

