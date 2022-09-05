#include "basic_alg.h"
#include <string.h>
extern "C"
{
#include "f2c.h"
#include "clapack.h"
}

using namespace std;

int32_t basic_alg::eigens(const double *p_matrix, int dim, double *eig_vec, double *eig_val)
{
    /*
    int dgeev_(
        char *jobvl, char *jobvr, 
        integer *n, doublereal *a, 
        integer *lda, doublereal *wr, 
        doublereal *wi, doublereal *vl, 
        integer *ldvl, doublereal *vr, 
        integer *ldvr, doublereal *work, 
        integer *lwork, integer *info
        );
    */
    /*  JOBVL   (input) CHARACTER*1 */
    /*          = 'N': left eigenvectors of A are not computed; */
    /*          = 'V': left eigenvectors of A are computed. */

    /*  JOBVR   (input) CHARACTER*1 */
    /*          = 'N': right eigenvectors of A are not computed; */
    /*          = 'V': right eigenvectors of A are computed. */

    /*  N       (input) INTEGER */
    /*          The order of the matrix A. N >= 0. */

    /*  A       (input/output) DOUBLE PRECISION array, dimension (LDA,N) */
    /*          On entry, the N-by-N matrix A. */
    /*          On exit, A has been overwritten. */

    /*  LDA     (input) INTEGER */
    /*          The leading dimension of the array A.  LDA >= max(1,N). */

    /*  WR      (output) DOUBLE PRECISION array, dimension (N) */
    /*  WI      (output) DOUBLE PRECISION array, dimension (N) */
    /*          WR and WI contain the real and imaginary parts, */
    /*          respectively, of the computed eigenvalues.  Complex */
    /*          conjugate pairs of eigenvalues appear consecutively */
    /*          with the eigenvalue having the positive imaginary part */
    /*          first. */

    /*  VL      (output) DOUBLE PRECISION array, dimension (LDVL,N) */
    /*          If JOBVL = 'V', the left eigenvectors u(j) are stored one */
    /*          after another in the columns of VL, in the same order */
    /*          as their eigenvalues. */
    /*          If JOBVL = 'N', VL is not referenced. */
    /*          If the j-th eigenvalue is real, then u(j) = VL(:,j), */
    /*          the j-th column of VL. */
    /*          If the j-th and (j+1)-st eigenvalues form a complex */
    /*          conjugate pair, then u(j) = VL(:,j) + i*VL(:,j+1) and */
    /*          u(j+1) = VL(:,j) - i*VL(:,j+1). */

    /*  LDVL    (input) INTEGER */
    /*          The leading dimension of the array VL.  LDVL >= 1; if */
    /*          JOBVL = 'V', LDVL >= N. */

    /*  VR      (output) DOUBLE PRECISION array, dimension (LDVR,N) */
    /*          If JOBVR = 'V', the right eigenvectors v(j) are stored one */
    /*          after another in the columns of VR, in the same order */
    /*          as their eigenvalues. */
    /*          If JOBVR = 'N', VR is not referenced. */
    /*          If the j-th eigenvalue is real, then v(j) = VR(:,j), */
    /*          the j-th column of VR. */
    /*          If the j-th and (j+1)-st eigenvalues form a complex */
    /*          conjugate pair, then v(j) = VR(:,j) + i*VR(:,j+1) and */
    /*          v(j+1) = VR(:,j) - i*VR(:,j+1). */

    /*  LDVR    (input) INTEGER */
    /*          The leading dimension of the array VR.  LDVR >= 1; if */
    /*          JOBVR = 'V', LDVR >= N. */

    /*  WORK    (workspace/output) DOUBLE PRECISION array, dimension (MAX(1,LWORK)) */
    /*          On exit, if INFO = 0, WORK(1) returns the optimal LWORK. */

    /*  LWORK   (input) INTEGER */
    /*          The dimension of the array WORK.  LWORK >= max(1,3*N), and */
    /*          if JOBVL = 'V' or JOBVR = 'V', LWORK >= 4*N.  For good */
    /*          performance, LWORK must generally be larger. */

    /*          If LWORK = -1, then a workspace query is assumed; the routine */
    /*          only calculates the optimal size of the WORK array, returns */
    /*          this value as the first entry of the WORK array, and no error */
    /*          message related to LWORK is issued by XERBLA. */

    /*  INFO    (output) INTEGER */
    /*          = 0:  successful exit */
    /*          < 0:  if INFO = -i, the i-th argument had an illegal value. */
    /*          > 0:  if INFO = i, the QR algorithm failed to compute all the */
    /*                eigenvalues, and no eigenvectors have been computed; */
    /*                elements i+1:N of WR and WI contain eigenvalues which */
    /*                have converged. */
    char calc_l = 'V';
    char calc_r = 'V'; // calculate both left and right if else the result different with matlab
    integer row = dim;
    integer col = dim;
    double *eig_val_i = new double[dim]; // imaginary
    double *eig_val_r = new double[dim]; // real
    double *mat = new double[dim * dim]; // stores the input matrix temporary
    memcpy(mat, p_matrix, sizeof(double) * dim * dim);
    double *ev_l = new double[dim * dim];
    double *ev_r = new double[dim * dim];
    double *work_space = new double[dim * 4];
    integer work_dim = 4 * dim;
    integer ret_info = 0;

    dgeev_(&calc_l, &calc_r, &row, mat, &col, eig_val_r, eig_val_i, ev_l, &col, ev_r, &col, work_space, &work_dim, &ret_info);
    if (ret_info != 0) goto ERROR;

    /*for (int i = 0; i < dim*dim; ++i)
    {
        printf("%lf,", ev_l[i]);
        if ((i + 1) % dim == 0) printf("\n");
    }*/

    *eig_val = eig_val_r[0];
    memcpy(eig_vec, ev_l, sizeof(double) * dim);

ERROR:
    delete[] eig_val_i;
    delete[] eig_val_r;
    delete[] mat;
    delete[] ev_l;
    delete[] ev_r;
    delete[] work_space;

    return (int32_t)ret_info;
}


int32_t basic_alg::inverse(const double *p_matrix, int dim, double *p_inv)
{
    integer row = dim;
    integer col = dim;
    integer ret_info = 0;

    double *mat = new double[dim * dim];
    integer *piviot = new integer[dim];
    memcpy(mat, p_matrix, sizeof(double) * dim * dim);
    double *work_space = new double[dim];
    integer work_dim = dim;

    /*  INFO    (output) INTEGER */
    /*          = 0:  successful exit */
    /*          < 0:  if INFO = -i, the i-th argument had an illegal value */
    /*          > 0:  if INFO = i, U(i,i) is exactly zero. The factorization */
    /*                has been completed, but the factor U is exactly */
    /*                singular, and division by zero will occur if it is used */
    /*                to solve a system of equations. */
    dgetrf_(&row, &col, mat, &row, piviot, &ret_info);
    if (ret_info != 0) goto ERROR;

    /*  INFO    (output) INTEGER */
    /*          = 0:  successful exit */
    /*          < 0:  if INFO = -i, the i-th argument had an illegal value */
    /*          > 0:  if INFO = i, U(i,i) is exactly zero; the matrix is */
    /*                singular and its inverse could not be computed. */
    dgetri_(&row, mat, &col, piviot, work_space, &work_dim, &ret_info);
    if (ret_info != 0) goto ERROR;

    memcpy(p_inv, mat, sizeof(double) * dim * dim);

ERROR:
    delete[] mat;
    delete[] piviot;
    delete[] work_space;

    return (int32_t)ret_info;
}