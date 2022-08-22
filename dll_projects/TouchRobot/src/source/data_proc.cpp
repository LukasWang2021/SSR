#ifdef _WIN_PLAT
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "data_proc.h"
#include "general_params.h"

#define MEAN_NUMS 9
//#define _TF_RECORD_
static FILE *fabc = NULL;
static int filter_enum_xyz = 0;
static int filter_enum_abc = 0;
static int filter_mean_xyz_nums = 10;
static int filter_mean_abc_nums = 10;

/*two-ordered butter-worth lowpass filter: fd=2Hz,fs=500Hz*/
static double bw_a_xyz[] =
    {1.000000000000000,-1.964465759657597,0.965086354987744};
static double bw_b_xyz[] =
    {0.000155148832536526,0.000310297665073052,0.000155148832536526};

static double bw_a_abc[] =
    {1.000000000000000,-1.964465759657597,0.965086354987744};
static double bw_b_abc[] =
    {0.000155148832536526,0.000310297665073052,0.000155148832536526};

static Vector3 *xyz_mean_hist = NULL;
static Vector3 *abc_mean_hist = NULL;


static void Data_Record(double *dat, int step);
static void Data_XyzFilter_MeanWindow(Vector3 *src, Vector3 *dst, int reset);
static void Data_AbcFilter_MeanWindow(Vector3 *src, Vector3 *dst, int reset);

static void Data_XyzFilter_LowPass(Vector3 *src, Vector3 *dst, int reset);
static void Data_AbcFilter_LowPass(Vector3 *src, Vector3 *dst, int reset);

static void Euler_to_RotationMatrix(const Vector3* v, RotationMatrix* rmx);


static void Data_XyzFilter_MeanWindow(Vector3 *src, Vector3 *dst, int reset)
{
    static int i = 0;
    static int cnt = 0;
    Vector3 sum = {0.0};

    if (reset == 1)
    {
        i = 0;
        cnt = 0;
        for (int n = 0; n < (filter_mean_xyz_nums - 1); n++)
        {
            xyz_mean_hist[n].x = 0;
            xyz_mean_hist[n].y = 0;
            xyz_mean_hist[n].z = 0;
        }
		return;
    }

    if(cnt >= (filter_mean_xyz_nums-1))
    {
        sum.x = src->x;
        sum.y = src->y;
        sum.z = src->z;

        for(int n=0; n<(filter_mean_xyz_nums-1); n++)
        {
            sum.x +=  xyz_mean_hist[n].x;
            sum.y +=  xyz_mean_hist[n].y;
            sum.z +=  xyz_mean_hist[n].z;
        }
        dst->x = sum.x/filter_mean_xyz_nums;
        dst->y = sum.y/filter_mean_xyz_nums;
        dst->z = sum.z/filter_mean_xyz_nums;

    }else{

        dst->x = src->x;
        dst->y = src->y;
        dst->z = src->z;
        cnt++;
    }

    xyz_mean_hist[i].x = dst->x;
    xyz_mean_hist[i].y = dst->y;
    xyz_mean_hist[i].z = dst->z;

    i = (i+1)%(filter_mean_xyz_nums-1);
}

static void Data_AbcFilter_MeanWindow(Vector3 *src, Vector3 *dst, int reset)
{
    static int i = 0;
    static int cnt = 0;

    Vector3 sum = {0.0};

    if (reset == 1)
    {
        i = 0;
        cnt = 0;
        for (int n = 0; n < (filter_mean_xyz_nums - 1); n++)
        {
            abc_mean_hist[n].x = 0;
            abc_mean_hist[n].y = 0;
            abc_mean_hist[n].z = 0;
        }
		return;
    }

    if(cnt >= (filter_mean_abc_nums-1))
    {
        if(src->x - abc_mean_hist[1].x > M_PI)
        {
            src->x -= (2*M_PI);
        }else if(src->x - abc_mean_hist[1].x < -M_PI){
            src->x += (2*M_PI);
        }

        if(src->y - abc_mean_hist[1].y > M_PI)
        {
            src->y -= (2*M_PI);
        }else if(src->y - abc_mean_hist[1].y < -M_PI){
            src->y += (2*M_PI);
        }

        if(src->z - abc_mean_hist[1].z > M_PI)
        {
            src->z -= (2*M_PI);
        }else if(src->z - abc_mean_hist[1].z < -M_PI){
            src->z += (2*M_PI);
        }
        sum.x = src->x;
        sum.y = src->y;
        sum.z = src->z;

        for(int n=0; n<(filter_mean_abc_nums-1); n++)
        {
            sum.x +=  abc_mean_hist[n].x;
            sum.y +=  abc_mean_hist[n].y;
            sum.z +=  abc_mean_hist[n].z;
        }
        dst->x = sum.x/filter_mean_abc_nums;
        dst->y = sum.y/filter_mean_abc_nums;
        dst->z = sum.z/filter_mean_abc_nums;

    }else{

        dst->x = src->x;
        dst->y = src->y;
        dst->z = src->z;
        cnt++;
    }

    abc_mean_hist[i].x = dst->x;
    abc_mean_hist[i].y = dst->y;
    abc_mean_hist[i].z = dst->z;

    i = (i+1)%(filter_mean_abc_nums-1);
}

static void Data_XyzFilter_LowPass(Vector3 *src, Vector3 *dst, int reset)
{
    static Vector3 src_hist[2] = {0.0};
    static Vector3 dst_hist[2] = {0.0};
    static int cnt = 0;

    if (reset == 1)
    {
        memset(src_hist, 0, 2 * sizeof(Vector3));
        memset(dst_hist, 0, 2 * sizeof(Vector3));
        cnt = 0;
        return;
    }

    if(cnt<2)
    {
        cnt++;
        dst->x = src->x;
        dst->y = src->y;
        dst->z = src->z;
    }else{
        dst->x = bw_b_xyz[0]*src->x + bw_b_xyz[1]*src_hist[1].x + bw_b_xyz[2]*src_hist[0].x - bw_a_xyz[1]*dst_hist[1].x - bw_a_xyz[2]*dst_hist[0].x;
        dst->y = bw_b_xyz[0]*src->y + bw_b_xyz[1]*src_hist[1].y + bw_b_xyz[2]*src_hist[0].y - bw_a_xyz[1]*dst_hist[1].y - bw_a_xyz[2]*dst_hist[0].y;
        dst->z = bw_b_xyz[0]*src->z + bw_b_xyz[1]*src_hist[1].z + bw_b_xyz[2]*src_hist[0].z - bw_a_xyz[1]*dst_hist[1].z - bw_a_xyz[2]*dst_hist[0].z;
    }
    src_hist[0].x = src_hist[1].x;
    src_hist[1].x = src->x;
    dst_hist[0].x = dst_hist[1].x;
    dst_hist[1].x = dst->x;

    src_hist[0].y = src_hist[1].y;
    src_hist[1].y = src->y;
    dst_hist[0].y = dst_hist[1].y;
    dst_hist[1].y = dst->y;

    src_hist[0].z = src_hist[1].z;
    src_hist[1].z = src->z;
    dst_hist[0].z = dst_hist[1].z;
    dst_hist[1].z = dst->z;
}

static void Data_AbcFilter_LowPass(Vector3 *src, Vector3 *dst, int reset)
{
    static Vector3 src_hist[2] = {0.0};
    static Vector3 dst_hist[2] = {0.0};
    static int cnt = 0;

    if (reset == 1)
    {
        memset(src_hist, 0, 2 * sizeof(Vector3));
        memset(dst_hist, 0, 2 * sizeof(Vector3));
        cnt = 0;
        return;
    }

    if(cnt<2)
    {
        cnt++;
        dst->x = src->x;
        dst->y = src->y;
        dst->z = src->z;

#ifdef _TF_RECORD_
		Data_Record(&src->x, 3);
#endif	

    }else{
        if(src->x - src_hist[1].x > M_PI)
        {
            src->x -= (2*M_PI);
        }else if(src->x - src_hist[1].x < -M_PI){
            src->x += (2*M_PI);
        }

        if(src->y - src_hist[1].y > M_PI)
        {
            src->y -= (2*M_PI);
        }else if(src->y - src_hist[1].y < -M_PI){
            src->y += (2*M_PI);
        }

        if(src->z - src_hist[1].z > M_PI)
        {
            src->z -= (2*M_PI);
        }else if(src->z - src_hist[1].z < -M_PI){
            src->z += (2*M_PI);
        }

#ifdef _TF_RECORD_
		Data_Record(&src->x, 3);
#endif	

        dst->x = bw_b_abc[0]*src->x + bw_b_abc[1]*src_hist[1].x + bw_b_abc[2]*src_hist[0].x - bw_a_abc[1]*dst_hist[1].x - bw_a_abc[2]*dst_hist[0].x;
        dst->y = bw_b_abc[0]*src->y + bw_b_abc[1]*src_hist[1].y + bw_b_abc[2]*src_hist[0].y - bw_a_abc[1]*dst_hist[1].y - bw_a_abc[2]*dst_hist[0].y;
        dst->z = bw_b_abc[0]*src->z + bw_b_abc[1]*src_hist[1].z + bw_b_abc[2]*src_hist[0].z - bw_a_abc[1]*dst_hist[1].z - bw_a_abc[2]*dst_hist[0].z;
    }

    src_hist[0].x = src_hist[1].x;
    src_hist[1].x = src->x;
    dst_hist[0].x = dst_hist[1].x;
    dst_hist[1].x = dst->x;

    src_hist[0].y = src_hist[1].y;
    src_hist[1].y = src->y;
    dst_hist[0].y = dst_hist[1].y;
    dst_hist[1].y = dst->y;

    src_hist[0].z = src_hist[1].z;
    src_hist[1].z = src->z;
    dst_hist[0].z = dst_hist[1].z;
    dst_hist[1].z = dst->z;

}

void RotationMatrix_to_Euler(const RotationMatrix *rotation_matrix_, Vector3 *abc)
{
//Z-Y-X
    double tmp = rotation_matrix_->matrix_[0][0] * rotation_matrix_->matrix_[0][0] \
                        + rotation_matrix_->matrix_[1][0] * rotation_matrix_->matrix_[1][0];


    if(fabs(fabs(rotation_matrix_->matrix_[2][0])-1)<0.000001)
    {
        if(rotation_matrix_->matrix_[2][0]<0)
        {
            abc->y = M_PI_2;
            abc->z = 0.0;
            abc->x = atan2(rotation_matrix_->matrix_[0][1],
                                                rotation_matrix_->matrix_[1][1]);
        }else{
            abc->y = -M_PI_2;
            abc->z = 0.0;
            abc->x = -atan2(rotation_matrix_->matrix_[0][1],
                                                rotation_matrix_->matrix_[1][1]);
        }
        return;
    }
    abc->y = atan2(-rotation_matrix_->matrix_[2][0],sqrt(tmp));//beta


    abc->z = atan2(rotation_matrix_->matrix_[1][0]/cos(abc->y),
                                        rotation_matrix_->matrix_[0][0]/cos(abc->y));//alpha

    abc->x = atan2(rotation_matrix_->matrix_[2][1]/cos(abc->y),
                                        rotation_matrix_->matrix_[2][2]/cos(abc->y));//gamma
 }


void Euler_to_RotationMatrix(const Vector3 *v, RotationMatrix *rmx)
{
    //Z-Y-X
    double sin_a = sin(v->z);//phi   alpha
    double cos_a = cos(v->z);
    double sin_b = sin(v->y);//theta beta
    double cos_b = cos(v->y);
    double sin_c = sin(v->x);//psi   gamma
    double cos_c = cos(v->x);

    memset(rmx,0,sizeof(RotationMatrix));
    rmx->matrix_[0][0] = cos_a*cos_b;
    rmx->matrix_[1][0] = sin_a*cos_b;
    rmx->matrix_[2][0] = -sin_b;
    rmx->matrix_[0][1] = cos_a*sin_b*sin_c-sin_a*cos_c;
    rmx->matrix_[1][1] = sin_a*sin_b*sin_c + cos_a*cos_c;
    rmx->matrix_[2][1] = cos_b*sin_c;
    rmx->matrix_[0][2] = cos_a*sin_b*cos_c+sin_a*sin_c;
    rmx->matrix_[1][2] = sin_a*sin_b*cos_c-cos_a*sin_c;
    rmx->matrix_[2][2] = cos_b*cos_c;
}


void Data_Filter(double *tmx, int reset)
{
    RotationMatrix rmx = {0};
    Vector3 pos_s = {0};
    Vector3 pos_tmp = {0};
    Vector3 pos_d = {0};
    Vector3 abc_s = {0};
    Vector3 abc_tmp = {0};
    Vector3 abc_d = {0};

    if (reset == 1)
    {
        Data_XyzFilter_LowPass(&pos_s, &pos_d, reset);
        Data_XyzFilter_MeanWindow(&pos_s, &pos_d, reset);

        Data_AbcFilter_LowPass(&abc_s, &abc_tmp, reset);
        Data_AbcFilter_MeanWindow(&abc_tmp, &abc_d, reset);
        return;
    }
        
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rmx.matrix_[j][i] = tmx[i * 4 + j];
        }
        *((double*)&pos_s.x + i) = tmx[12 + i];
    }

#ifdef _TF_RECORD_
    Data_Record(&pos_s.x,1);// xyz src
#endif

    RotationMatrix_to_Euler(&rmx, &abc_s);

#ifdef _TF_RECORD_
    Data_Record(&abc_s.x, 2);// abc src
#endif

    switch(filter_enum_xyz)
    {
    case BUTTOR_WORTH_LP:
        Data_XyzFilter_LowPass(&pos_s, &pos_d, reset);
        break;
    case MEAN_WINDOW:
        Data_XyzFilter_MeanWindow(&pos_s, &pos_d, reset);
        break;
    case BOTH:
        Data_XyzFilter_LowPass(&pos_s, &pos_tmp, reset);
        Data_XyzFilter_MeanWindow(&pos_tmp, &pos_d, reset);
        break;
    default:
        break;
    }

    switch(filter_enum_abc)
    {
    case BUTTOR_WORTH_LP:
        Data_AbcFilter_LowPass(&abc_s, &abc_d, reset);// abc normilize
        break;
    case MEAN_WINDOW:
        Data_AbcFilter_MeanWindow(&abc_s, &abc_d, reset);
        break;
    case BOTH:
        Data_AbcFilter_LowPass(&abc_s, &abc_tmp, reset);
        Data_AbcFilter_MeanWindow(&abc_tmp, &abc_d, reset);
        break;
    default:
        break;
    }

#ifdef _TF_RECORD_
    Data_Record(&abc_d.x, 4);
#endif

    Euler_to_RotationMatrix(&abc_d, &rmx);
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            tmx[i*4+j] = rmx.matrix_[j][i];
        }
        tmx[12+i] = *((double*)&pos_d.x+i);
    }

#ifdef _TF_RECORD_
    Data_Record(&pos_d.x,5);
#endif
}

void Data_Proc_Init(void *param)
{
    double Omega;
    TouchParams_t*param_p = (TouchParams_t*)param;
#ifdef _TF_RECORD_
    fabc = fopen("TouchData.csv","w+");
#endif
    filter_enum_xyz = param_p->filter_enum_xyz;
    filter_enum_abc = param_p->filter_enum_abc;

    if(filter_enum_xyz == BUTTOR_WORTH_LP || filter_enum_xyz == BOTH)
    {
        Omega = tan(M_PI*param_p->filter_fd_xyz/param_p->freq_touch);
        bw_b_xyz[0] = Omega*Omega;
        bw_b_xyz[1] = 2*Omega*Omega;
        bw_b_xyz[2] = Omega*Omega;

        bw_a_xyz[0] = 1 + sqrt(2)*Omega + Omega*Omega;
        bw_a_xyz[1] = 2*Omega*Omega - 2;
        bw_a_xyz[2] = 1 - sqrt(2)*Omega + Omega*Omega;

        bw_b_xyz[0] = bw_b_xyz[0]/bw_a_xyz[0];
        bw_b_xyz[1] = bw_b_xyz[1]/bw_a_xyz[0];
        bw_b_xyz[2] = bw_b_xyz[2]/bw_a_xyz[0];
        bw_a_xyz[1] = bw_a_xyz[1]/bw_a_xyz[0];
        bw_a_xyz[2] = bw_a_xyz[2]/bw_a_xyz[0];
        bw_a_xyz[0] = 1.0;
    }
    if(filter_enum_abc == BUTTOR_WORTH_LP || filter_enum_abc == BOTH)
    {
        Omega = tan(M_PI*param_p->filter_fd_abc/param_p->freq_touch);
        bw_b_abc[0] = Omega*Omega;
        bw_b_abc[1] = 2*Omega*Omega;
        bw_b_abc[2] = Omega*Omega;

        bw_a_abc[0] = 1 + sqrt(2)*Omega + Omega*Omega;
        bw_a_abc[1] = 2*Omega*Omega - 2;
        bw_a_abc[2] = 1 - sqrt(2)*Omega + Omega*Omega;

        bw_b_abc[0] = bw_b_abc[0]/bw_a_abc[0];
        bw_b_abc[1] = bw_b_abc[1]/bw_a_abc[0];
        bw_b_abc[2] = bw_b_abc[2]/bw_a_abc[0];
        bw_a_abc[1] = bw_a_abc[1]/bw_a_abc[0];
        bw_a_abc[2] = bw_a_abc[2]/bw_a_abc[0];
        bw_a_abc[0] = 1.0;
    }

    filter_mean_xyz_nums = param_p->filter_mean_xyz_nums;
    filter_mean_abc_nums = param_p->filter_mean_abc_nums;
    xyz_mean_hist = (Vector3*)malloc(filter_mean_xyz_nums*sizeof(Vector3));
    abc_mean_hist = (Vector3*)malloc(filter_mean_abc_nums*sizeof(Vector3));

}
void Data_Record(double *dat, int step)
{
    switch(step)
    {
    case 0:
        fprintf(fabc, "%lf,%lf,%lf,%lf,"
                      "%lf,%lf,%lf,%lf,"
                      "%lf,%lf,%lf,%lf,"
                      "%lf,%lf,%lf,%lf,",
                            dat[0],dat[1],dat[2],dat[3],
                            dat[4],dat[5],dat[6],dat[7],
                            dat[8],dat[9],dat[10],dat[11],
                            dat[12],dat[13],dat[14],dat[15]);
        break;
    case 1:
    case 2:
    case 3:
	case 4:
        fprintf(fabc, "%lf,%lf,%lf,",dat[0],dat[1],dat[2]);
        break;
    case 5:
        fprintf(fabc, "%lf,%lf,%lf\n",dat[0],dat[1],dat[2]);
        break;
    default:
        break;
    }
    fflush(fabc);
}
void Data_Proc_End(void)
{
#ifdef _TF_RECORD_
    fclose(fabc);
#endif

    free(xyz_mean_hist);
    free(abc_mean_hist);
}

void multiply44(const Matrix44* left_matrix, const Matrix44* right_matrix, Matrix44* result_matrix)
{
    result_matrix->matrix_[0][0] = left_matrix->matrix_[0][0] * right_matrix->matrix_[0][0]
        + left_matrix->matrix_[0][1] * right_matrix->matrix_[1][0]
        + left_matrix->matrix_[0][2] * right_matrix->matrix_[2][0]
        + left_matrix->matrix_[0][3] * right_matrix->matrix_[3][0];

    result_matrix->matrix_[0][1] = left_matrix->matrix_[0][0] * right_matrix->matrix_[0][1]
        + left_matrix->matrix_[0][1] * right_matrix->matrix_[1][1]
        + left_matrix->matrix_[0][2] * right_matrix->matrix_[2][1]
        + left_matrix->matrix_[0][3] * right_matrix->matrix_[3][1];

    result_matrix->matrix_[0][2] = left_matrix->matrix_[0][0] * right_matrix->matrix_[0][2]
        + left_matrix->matrix_[0][1] * right_matrix->matrix_[1][2]
        + left_matrix->matrix_[0][2] * right_matrix->matrix_[2][2]
        + left_matrix->matrix_[0][3] * right_matrix->matrix_[3][2];

    result_matrix->matrix_[0][3] = left_matrix->matrix_[0][0] * right_matrix->matrix_[0][3]
        + left_matrix->matrix_[0][1] * right_matrix->matrix_[1][3]
        + left_matrix->matrix_[0][2] * right_matrix->matrix_[2][3]
        + left_matrix->matrix_[0][3] * right_matrix->matrix_[3][3];

    result_matrix->matrix_[1][0] = left_matrix->matrix_[1][0] * right_matrix->matrix_[0][0]
        + left_matrix->matrix_[1][1] * right_matrix->matrix_[1][0]
        + left_matrix->matrix_[1][2] * right_matrix->matrix_[2][0]
        + left_matrix->matrix_[1][3] * right_matrix->matrix_[3][0];

    result_matrix->matrix_[1][1] = left_matrix->matrix_[1][0] * right_matrix->matrix_[0][1]
        + left_matrix->matrix_[1][1] * right_matrix->matrix_[1][1]
        + left_matrix->matrix_[1][2] * right_matrix->matrix_[2][1]
        + left_matrix->matrix_[1][3] * right_matrix->matrix_[3][1];

    result_matrix->matrix_[1][2] = left_matrix->matrix_[1][0] * right_matrix->matrix_[0][2]
        + left_matrix->matrix_[1][1] * right_matrix->matrix_[1][2]
        + left_matrix->matrix_[1][2] * right_matrix->matrix_[2][2]
        + left_matrix->matrix_[1][3] * right_matrix->matrix_[3][2];

    result_matrix->matrix_[1][3] = left_matrix->matrix_[1][0] * right_matrix->matrix_[0][3]
        + left_matrix->matrix_[1][1] * right_matrix->matrix_[1][3]
        + left_matrix->matrix_[1][2] * right_matrix->matrix_[2][3]
        + left_matrix->matrix_[1][3] * right_matrix->matrix_[3][3];

    result_matrix->matrix_[2][0] = left_matrix->matrix_[2][0] * right_matrix->matrix_[0][0]
        + left_matrix->matrix_[2][1] * right_matrix->matrix_[1][0]
        + left_matrix->matrix_[2][2] * right_matrix->matrix_[2][0]
        + left_matrix->matrix_[2][3] * right_matrix->matrix_[3][0];

    result_matrix->matrix_[2][1] = left_matrix->matrix_[2][0] * right_matrix->matrix_[0][1]
        + left_matrix->matrix_[2][1] * right_matrix->matrix_[1][1]
        + left_matrix->matrix_[2][2] * right_matrix->matrix_[2][1]
        + left_matrix->matrix_[2][3] * right_matrix->matrix_[3][1];

    result_matrix->matrix_[2][2] = left_matrix->matrix_[2][0] * right_matrix->matrix_[0][2]
        + left_matrix->matrix_[2][1] * right_matrix->matrix_[1][2]
        + left_matrix->matrix_[2][2] * right_matrix->matrix_[2][2]
        + left_matrix->matrix_[2][3] * right_matrix->matrix_[3][2];

    result_matrix->matrix_[2][3] = left_matrix->matrix_[2][0] * right_matrix->matrix_[0][3]
        + left_matrix->matrix_[2][1] * right_matrix->matrix_[1][3]
        + left_matrix->matrix_[2][2] * right_matrix->matrix_[2][3]
        + left_matrix->matrix_[2][3] * right_matrix->matrix_[3][3];

    result_matrix->matrix_[3][0] = left_matrix->matrix_[3][0] * right_matrix->matrix_[0][0]
        + left_matrix->matrix_[3][1] * right_matrix->matrix_[1][0]
        + left_matrix->matrix_[3][2] * right_matrix->matrix_[2][0]
        + left_matrix->matrix_[3][3] * right_matrix->matrix_[3][0];

    result_matrix->matrix_[3][1] = left_matrix->matrix_[3][0] * right_matrix->matrix_[0][1]
        + left_matrix->matrix_[3][1] * right_matrix->matrix_[1][1]
        + left_matrix->matrix_[3][2] * right_matrix->matrix_[2][1]
        + left_matrix->matrix_[3][3] * right_matrix->matrix_[3][1];

    result_matrix->matrix_[3][2] = left_matrix->matrix_[3][0] * right_matrix->matrix_[0][2]
        + left_matrix->matrix_[3][1] * right_matrix->matrix_[1][2]
        + left_matrix->matrix_[3][2] * right_matrix->matrix_[2][2]
        + left_matrix->matrix_[3][3] * right_matrix->matrix_[3][2];

    result_matrix->matrix_[3][3] = left_matrix->matrix_[3][0] * right_matrix->matrix_[0][3]
        + left_matrix->matrix_[3][1] * right_matrix->matrix_[1][3]
        + left_matrix->matrix_[3][2] * right_matrix->matrix_[2][3]
        + left_matrix->matrix_[3][3] * right_matrix->matrix_[3][3];
}