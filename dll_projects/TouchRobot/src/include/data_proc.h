#ifndef DATA_PROC_H_
#define DATA_PROC_H_

#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
    double x;
    double y;
    double z;
}Vector3;

typedef struct
{
    double matrix_[3][3];
}Matrix33;

typedef struct
{
    double matrix_[4][4];
}Matrix44;

typedef struct
{
    double a_;
    double b_;
    double c_;
}Euler;

typedef struct
{
    double x_;
    double y_;
    double z_;
    double a_;//alpha
    double b_;//beta
    double c_;//gamma
}PointEuler;

typedef enum{
    BUTTOR_WORTH_LP = 0,
    MEAN_WINDOW = 1,
    BOTH = 2,
}FilterEnum_e;

typedef Matrix33 RotationMatrix;


/**
 * @brief Data filter process of transmatrixs.
 * @param [in] transmatrix address
 * @param [in] reset flag
 * @retval void.
 */
void Data_Filter(double *tmx, int reset);

/**
 * @brief Data filter init process.
 * @param [in] which filter, 0-butter, 1-mean_window, 2-both.
 * @retval void.
 */
void Data_Proc_Init(void *params);/*which: 0-butter, 1-mean_window, 2-both*/

/**
 * @brief Data filter end process.
 * @retval void.
 */
void Data_Proc_End(void);

void RotationMatrix_to_Euler(const RotationMatrix* rotation_matrix_, Vector3* abc);
#ifdef __cplusplus
}
#endif

#endif
