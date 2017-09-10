#include "HomographyDecomposer.h"
#include <iostream>
#include <complex>
#include <fstream>
#include <QDebug>

using namespace std;

HomographyDecomposer::HomographyDecomposer()
{
    homography = Mat(3, 3, CV_64F);
    transform = Mat(4, 4, CV_64F);
    transform.setTo(0);
	cameraIntrinsicParameters = Mat(3, 3, CV_64F);
	decompMethod = ROTATIONAL;
}

void HomographyDecomposer::setMethod(enum DecompMethods decompMethod)
{
	this->decompMethod = decompMethod;
}

void HomographyDecomposer::setHomography(Mat &homography)
{
	homography.copyTo(this->homography);
}

void HomographyDecomposer::setCameraIntrinsicParameters(Mat &cameraIntrinsicParameters)
{
	cameraIntrinsicParameters.copyTo(this->cameraIntrinsicParameters);
}

cv::Point3f HomographyDecomposer::getEulerAngles(cv::Mat transform)
{
	cv::Point3f angles;
	angles.x = atan2(transform.at<double>(2, 0), transform.at<double>(2, 1)) * (180.0 / 3.1416); // Phi, rotation in X
	angles.y = acos(transform.at<double>(2, 2)) * (180.0 / 3.1416); // Theta, rotation in Y
	angles.z = -atan2(transform.at<double>(0, 2), transform.at<double>(1, 2)) * (180.0 / 3.1416); // Psi, rotation in Z
	return angles;
}

cv::Point3f HomographyDecomposer::getAxisAngles(cv::Mat transform)
{
	cv::Point3f angles;
	angles.z = atan2(transform.at<double>(1, 0), transform.at<double>(0, 0)) * (180.0 / 3.1416); // Alpha, rotation in z
	angles.y = atan2(-transform.at<double>(2, 0), sqrt(pow(transform.at<double>(2, 1), 2) + pow(transform.at<double>(2, 2), 2)) ) * (180.0 / 3.1416); // Betha, rotation in Y
	angles.x = atan2(transform.at<double>(2, 1), transform.at<double>(2, 2)) * (180.0 / 3.1416); // Gamma, rotation in x
	return angles;
}

cv::Point3f HomographyDecomposer::getTranslation(cv::Mat transform)
{
	cv::Point3f translation;
	translation.x = transform.at<double>(0, 3);
	translation.y = transform.at<double>(1, 3);
	translation.z = transform.at<double>(2, 3);
	return translation;
}

void HomographyDecomposer::decompose()
{
	switch (decompMethod)
	{
		case ANALYTICAL: decomposeAnalytical(); break;
		case ROTATIONAL: decomposeRotational(); break;
	}
}

void HomographyDecomposer::decomposeAnalytical()
{
//	CvMat K_temp = cameraIntrinsicParameters;
//	CvMat G_temp = homography;
//	CvMat *K = &K_temp;
//	CvMat *G = &G_temp;

//	double r11, r12, r13, r21, r22, r23, r31, r32, r33;
//	double tx, ty, tz;

//	CvMat *H_hat = cvCreateMat(3, 3, CV_64F);
//	CvMat *K_inv = cvCreateMat(3, 3, CV_64F);

//	// Calculate ^H (H hat)

//	// K^-1
//	cvInvert(K, K_inv, CV_LU);
//	// ^H = ^K^-1 * G * ^K assuming ^K = K
	
//	cvGEMM(K_inv, G, 1, 0, 0, H_hat);
//	cvGEMM(H_hat, K, 1, 0, 0, H_hat);
	
//	// 2. Calculate H = ^H / Gamma
	
//	// M = ^Htranspose * ^H
//	CvMat *M = cvCreateMat(3, 3, CV_64F);
//	cvGEMM(H_hat, H_hat, 1, 0, 0, M, CV_GEMM_A_T);
	
//	// Make it easier to read
//	double m11, m12, m13, m21, m22, m23, m31, m32, m33;
//	m11 = CV_MAT_ELEM(*M, double, 0, 0); m12 = CV_MAT_ELEM(*M, double, 0, 1); m13 = CV_MAT_ELEM(*M, double, 0, 2);
//	m21 = CV_MAT_ELEM(*M, double, 1, 0); m22 = CV_MAT_ELEM(*M, double, 1, 1); m23 = CV_MAT_ELEM(*M, double, 1, 2);
//	m31 = CV_MAT_ELEM(*M, double, 2, 0); m32 = CV_MAT_ELEM(*M, double, 2, 1); m33 = CV_MAT_ELEM(*M, double, 2, 2);

//	// Coeficients of l^3 + a2 * l^2 + a1 * l + a0 = 0
//	double a2, a1, a0;
//	a2 = -(m11 + m22 + m33);
//	a1 = m11 * m22 + m11 * m33 + m22 * m33 - (pow(m12, 2) + pow(m13, 2) + pow(m23, 2));
//	a0 = pow(m12, 2) * m33 + pow(m13, 2) * m22 + pow(m23, 2) * m11 - m11 * m22 * m33 - 2 * m12 * m13 * m23;
	
//	// Notation: c_VAR means it's complex number
//	// i = (-1)^(1/2)
//	complex<double> c_i = sqrt(complex<double>(-1));
//	double Q, R;
//	Q = (3 * a1 - pow(a2, 2)) / 9;
//	R = (9 * a2 * a1 - 27 * a0 - 2 * pow(a2, 3)) / 54;

//	// Intermedium term: c_QR = (Q^3 + R^2)^(1/2);
//	complex<double> c_QR = sqrt(complex<double>(pow(Q, 3) + pow(R, 2)));
//	complex<double> c_S = pow(R + c_QR, 1 / 3.0);
//	complex<double> c_T = pow(R - c_QR, 1 / 3.0);

//	// Complex to check that effectively imaginary part is 0 in all cases
//	// This can be more efficient... but it's not significative
//	complex<double> c_lambda1, c_lambda2, c_lambda3;
//	c_lambda1 = -a2 / 3 + (c_S + c_T);
//	c_lambda2 = -a2 / 3 - 0.5 * (c_S + c_T) - (sqrt(3.0) / 2) * (c_S - c_T) * c_i;
//	c_lambda3 = -a2 / 3 - 0.5 * (c_S + c_T) + (sqrt(3.0) / 2) * (c_S - c_T) * c_i;

//	//cout << "Vars a2:" << a2 << " a1:" << a1 << " a0:" << a0 << " Q:" << Q << " R:" << R << "\r\n";
//	//cout << "Imaginary Vars c_QR: " << c_QR << ", c_S" << c_S << ", c_T" << c_T << "\r\n";
//	//cout << "Imaginary Check: " << c_lambda1 << ", " << c_lambda2 << ", " << c_lambda3 << "\r\n";
	
//	double lambda = real(c_lambda2);
//	double gamma = sqrt(lambda);

//	//fstream debugFile; debugFile.open("DebugMainLoop.txt", ios::app);
//	//debugFile << "mXY," << m11 << "," << m12 << "," << m13 << "," << m21 << "," << m22 << "," << m23 << "," << m31 << "," << m32 << "," << m33 << ",";
//	//debugFile << "a0a1a2QR," << a0 << "," << a1 << "," << a2 << "," << Q << "," << R << ",";
//	//debugFile << "QR_S_T," << c_QR.real() << "," << c_QR.imag() << "," << c_S.real() << "," << c_S.imag() << "," << c_T.real() << "," << c_T.imag() << ",";
//	//debugFile << "Lambdas," << c_lambda1.real() << "," << c_lambda1.imag() << "," << c_lambda2.real() << "," << c_lambda2.imag() << "," << c_lambda3.real() << "," << c_lambda3.imag() << ",";
//	//debugFile << "LambdaYGamma," << lambda << "," << gamma << ",";
//	//debugFile << "\r\n";
//	//debugFile.close();
	
//	// Finally, H = ^H / Gamma
//	CvMat *H = cvCreateMat(3, 3, CV_64F);
//	cvScale(H_hat, H, (1 / gamma));

//	// 3. Estimate S = Htranspose * H - I
//	CvMat *S = cvCreateMat(3, 3, CV_64F);
//	CvMat *I = cvCreateMat(3, 3, CV_64F);
//	cvSetIdentity(I);
//	cvGEMM(H, H, 1, I, -1, S, CV_GEMM_A_T);

//	// Make it easier to read
//	double s11, s12, s13, s21, s22, s23, s31, s32, s33;
//	s11 = CV_MAT_ELEM(*S, double, 0, 0); s12 = CV_MAT_ELEM(*S, double, 0, 1); s13 = CV_MAT_ELEM(*S, double, 0, 2);
//	s21 = CV_MAT_ELEM(*S, double, 1, 0); s22 = CV_MAT_ELEM(*S, double, 1, 1); s23 = CV_MAT_ELEM(*S, double, 1, 2);
//	s31 = CV_MAT_ELEM(*S, double, 2, 0); s32 = CV_MAT_ELEM(*S, double, 2, 1); s33 = CV_MAT_ELEM(*S, double, 2, 2);

//	// 4. Calculate opposites of minors
//	double Ms11 = pow(s23, 2) - s22 * s33;
//	double Ms22 = pow(s13, 2) - s11 * s33;
//	double Ms33 = pow(s12, 2) - s11 * s22;
	
//	double Ms12 = s23 * s13 - s12 * s33;
//	double Ms13 = s22 * s13 - s12 * s23;
//	double Ms23 = s12 * s13 - s11 * s23;
	
//	// 5. Obtain epsilon values using sign functions
//	double eps12 = Ms12 >= 0 ? 1 : -1;
//	double eps13 = Ms13 >= 0 ? 1 : -1;
//	double eps23 = Ms23 >= 0 ? 1 : -1;
//	//double eps12 = Ms12 >= 0 ? -1 : -1;
//	//double eps13 = Ms13 >= 0 ? -1 : -1;
//	//double eps23 = Ms23 >= 0 ? -1 : -1;

//	CvMat *n_acute_a = cvCreateMat(3, 1, CV_64F);
//	CvMat *n_acute_b = cvCreateMat(3, 1, CV_64F);
//	CvMat *n_a = cvCreateMat(3, 1, CV_64F);
//	CvMat *n_b = cvCreateMat(3, 1, CV_64F);
//	CvMat *t_ref_a = cvCreateMat(3, 1, CV_64F);
//	CvMat *t_ref_b = cvCreateMat(3, 1, CV_64F);
//	CvMat *R_a = cvCreateMat(3, 3, CV_64F);
//	CvMat *R_b = cvCreateMat(3, 3, CV_64F);
//	CvMat *t_a = cvCreateMat(3, 1, CV_64F);
//	CvMat *t_b = cvCreateMat(3, 1, CV_64F);

//	// 6. Calculate v
//	// Trace of S
//	double S_trace = cvTrace(S).val[0];

//	double v = 2 * sqrt(1 + S_trace - Ms11 - Ms22 - Ms33);
//	double ro_squared = 2 + S_trace + v;
	
//	// 7. Calculate the squared norm of te
//	double te_norm_squared = 2 + S_trace - v;

//	//fstream debugFile; debugFile.open("DebugMainLoop2.txt", ios::app);
//	//debugFile << "sXY," << s11 << "," << s12 << "," << s13 << "," << s21 << "," << s22 << "," << s23 << "," << s31 << "," << s32 << "," << s33 << ",";
//	//debugFile << "mXY," << Ms11 << "," << Ms12 << "," << Ms13 << "," << Ms22 << "," << Ms23 << "," << Ms33 << ",";
//	//debugFile << "epsX," << eps12 << "," << eps13 << "," << eps23 << ",";
//	//debugFile << "trace(s),v,ro,norm(te)," << S_trace << "," << v << "," << ro_squared << "," << te_norm_squared << ",";
//	//debugFile << "\r\n";
//	//debugFile.close();


//	if ((abs(s11) >= abs(s22)) && (abs(s11) >= abs(s33)))
//	//if (0)
//	{
//		// 8. Calculate normal vector n
//		double n_x, n_y, n_z;
//		// Solution a
//		n_x = s11;
//		n_y = s12 + sqrt(Ms33);
//		n_z = s13 + eps23 * sqrt(Ms22);
//		CV_MAT_ELEM(*n_acute_a, double, 0, 0) = n_x;
//		CV_MAT_ELEM(*n_acute_a, double, 1, 0) = n_y;
//		CV_MAT_ELEM(*n_acute_a, double, 2, 0) = n_z;
//		// Solution b
//		n_y = s12 - sqrt(Ms33);
//		n_z = s13 - eps23 * sqrt(Ms22);
//		CV_MAT_ELEM(*n_acute_b, double, 0, 0) = n_x;
//		CV_MAT_ELEM(*n_acute_b, double, 1, 0) = n_y;
//		CV_MAT_ELEM(*n_acute_b, double, 2, 0) = n_z;
		
//		// 9. Get the translation vector in reference frame
//		double t_x, t_y, t_z, factor_1, factor_2;
//		// Solution a
//		factor_1 = cvNorm(n_acute_a) / (2 * s11);
//		factor_2 = te_norm_squared / (2 * cvNorm(n_acute_a));
//		t_x = factor_1 * s11 - factor_2 * s11;
//		t_y = factor_1 * (s12 - sqrt(Ms33)) - factor_2 * (s12 + sqrt(Ms33));
//		t_z = factor_1 * (s13 - eps23 * sqrt(Ms22)) - factor_2 * (s13 + eps23 * sqrt(Ms22));
//		CV_MAT_ELEM(*t_ref_a, double, 0, 0) = t_x;
//		CV_MAT_ELEM(*t_ref_a, double, 1, 0) = t_y;
//		CV_MAT_ELEM(*t_ref_a, double, 2, 0) = t_z;
//		// Solution b
//		factor_1 = cvNorm(n_acute_b) / (2 * s11);
//		factor_2 = te_norm_squared / (2 * cvNorm(n_acute_b));
//		t_x = factor_1 * s11 - factor_2 * s11;
//		t_y = factor_1 * (s12 + sqrt(Ms33)) - factor_2 * (s12 - sqrt(Ms33));
//		t_z = factor_1 * (s13 + eps23 * sqrt(Ms22)) - factor_2 * (s13 - eps23 * sqrt(Ms22));
//		CV_MAT_ELEM(*t_ref_b, double, 0, 0) = t_x;
//		CV_MAT_ELEM(*t_ref_b, double, 1, 0) = t_y;
//		CV_MAT_ELEM(*t_ref_b, double, 2, 0) = t_z;

//		cvScale(n_acute_a, n_a, 1/cvNorm(n_acute_a));
//		cvScale(n_acute_b, n_b, 1/cvNorm(n_acute_b));
//		// Solution a
//		cvGEMM(t_ref_a, n_a, -2 / v, I, 1, R_a, CV_GEMM_B_T);
//		cvGEMM(H, R_a, 1, 0, 0, R_a);
//		cvGEMM(R_a, t_ref_a, 1, 0, 0, t_a);
//		// Solution b
//		cvGEMM(t_ref_b, n_b, -2 / v, I, 1, R_b, CV_GEMM_B_T);
//		cvGEMM(H, R_b, 1, 0, 0, R_b);
//		cvGEMM(R_b, t_ref_b, 1, 0, 0, t_b);

//		r11 = CV_MAT_ELEM(*R_a, double, 0, 0);
//		r21 = CV_MAT_ELEM(*R_a, double, 1, 0);
//		r31 = CV_MAT_ELEM(*R_a, double, 2, 0);
//		r12 = CV_MAT_ELEM(*R_a, double, 0, 1);
//		r22 = CV_MAT_ELEM(*R_a, double, 1, 1);
//		r32 = CV_MAT_ELEM(*R_a, double, 2, 1);
//		r13 = CV_MAT_ELEM(*R_a, double, 0, 2);
//		r23 = CV_MAT_ELEM(*R_a, double, 1, 2);
//		r33 = CV_MAT_ELEM(*R_a, double, 2, 2);
//		tx = CV_MAT_ELEM(*t_b, double, 0, 0);
//		ty = CV_MAT_ELEM(*t_b, double, 1, 0);
//		tz = CV_MAT_ELEM(*t_b, double, 2, 0);
//	}
//	else if ((abs(s22) >= abs(s11)) && (abs(s22) >= abs(s33)))
//	//else if (true)
//	{
//		//fstream f; f.open("VariablesDebug.txt", ios::app);

//		// 8. Calculate normal vector n
//		double n_x, n_y, n_z;
//		// Solution a
//		n_x = s12 + sqrt(Ms33);
//		n_y = s22;
//		n_z = s23 - eps13 * sqrt(Ms11);

//		//f << n_x << "," << n_y << "," << n_z << ",";

//		CV_MAT_ELEM(*n_acute_a, double, 0, 0) = n_x;
//		CV_MAT_ELEM(*n_acute_a, double, 1, 0) = n_y;
//		CV_MAT_ELEM(*n_acute_a, double, 2, 0) = n_z;
//		// Solution b
//		n_x = s12 - sqrt(Ms33);
//		n_z = s23 + eps13 * sqrt(Ms11);
//		CV_MAT_ELEM(*n_acute_b, double, 0, 0) = n_x;
//		CV_MAT_ELEM(*n_acute_b, double, 1, 0) = n_y;
//		CV_MAT_ELEM(*n_acute_b, double, 2, 0) = n_z;
		
//		// 9. Get the translation vector in reference frame
//		double t_x, t_y, t_z, factor_1, factor_2;
//		// Solution a
//		factor_1 = cvNorm(n_acute_a) / (2 * s22);
//		factor_2 = te_norm_squared / (2 * cvNorm(n_acute_a));

//		t_x = factor_1 * (s12 - sqrt(Ms33)) - factor_2 * (s12 + sqrt(Ms33));
//		t_y = factor_1 * s22 - factor_2 * s22;
//		t_z = factor_1 * (s23 + eps13 * sqrt(Ms11)) - factor_2 * (s23 - eps13 * sqrt(Ms11));

//		//f << t_x << "," << t_y << "," << t_z << ",";
//		//f << factor_1 << "," << factor_2 << ",";

//		CV_MAT_ELEM(*t_ref_a, double, 0, 0) = t_x;
//		CV_MAT_ELEM(*t_ref_a, double, 1, 0) = t_y;
//		CV_MAT_ELEM(*t_ref_a, double, 2, 0) = t_z;

//		// Solution b
//		factor_1 = cvNorm(n_acute_b) / (2 * s22);
//		factor_2 = te_norm_squared / (2 * cvNorm(n_acute_b));
//		t_x = factor_1 * (s12 + sqrt(Ms33)) - factor_2 * (s12 - sqrt(Ms33));
//		t_y = factor_1 * s22 - factor_2 * s22;
//		t_z = factor_1 * (s23 - eps13 * sqrt(Ms11)) - factor_2 * (s23 + eps13 * sqrt(Ms11));
//		CV_MAT_ELEM(*t_ref_b, double, 0, 0) = t_x;
//		CV_MAT_ELEM(*t_ref_b, double, 1, 0) = t_y;
//		CV_MAT_ELEM(*t_ref_b, double, 2, 0) = t_z;

//		cvScale(n_acute_a, n_a, 1/cvNorm(n_acute_a));
//		cvScale(n_acute_b, n_b, 1/cvNorm(n_acute_b));

//		// Solution a
//		cvGEMM(t_ref_a, n_a, -2 / v, I, 1, R_a, CV_GEMM_B_T);
//		cvGEMM(H, R_a, 1, 0, 0, R_a);
//		cvGEMM(R_a, t_ref_a, 1, 0, 0, t_a);
		
//		// Solution b
//		cvGEMM(t_ref_b, n_b, -2 / v, I, 1, R_b, CV_GEMM_B_T);
//		cvGEMM(H, R_b, 1, 0, 0, R_b);
//		cvGEMM(R_b, t_ref_b, 1, 0, 0, t_b);

//		r11 = CV_MAT_ELEM(*R_a, double, 0, 0);
//		r21 = CV_MAT_ELEM(*R_a, double, 1, 0);
//		r31 = CV_MAT_ELEM(*R_a, double, 2, 0);
//		r12 = CV_MAT_ELEM(*R_a, double, 0, 1);
//		r22 = CV_MAT_ELEM(*R_a, double, 1, 1);
//		r32 = CV_MAT_ELEM(*R_a, double, 2, 1);
//		r13 = CV_MAT_ELEM(*R_a, double, 0, 2);
//		r23 = CV_MAT_ELEM(*R_a, double, 1, 2);
//		r33 = CV_MAT_ELEM(*R_a, double, 2, 2);
//		tx = CV_MAT_ELEM(*t_a, double, 0, 0);
//		ty = CV_MAT_ELEM(*t_a, double, 1, 0);
//		tz = CV_MAT_ELEM(*t_a, double, 2, 0);
	
//		//f << r11 << "," << r21 << "," << r31 << ",";
//		//f << r12 << "," << r22 << "," << r32 << ",";
//		//f << r13 << "," << r23 << "," << r33 << ",";
//		//f << tx << "," << ty << "," << tz << ",";

//		//f << "\r\n";
//		//f.close();
//		//std::cout << "Dot product n . o " << (r11*r12 + r21*r22 + r31*r32) << "\r\n";
//	}
//	else
//	{
//		// 8. Calculate normal vector n
//		double n_x, n_y, n_z;
//		// Solution a
//		n_x = s13 + eps12 * sqrt(Ms22);
//		n_y = s23 + sqrt(Ms11);
//		n_z = s33;
//		CV_MAT_ELEM(*n_acute_a, double, 0, 0) = n_x;
//		CV_MAT_ELEM(*n_acute_a, double, 1, 0) = n_y;
//		CV_MAT_ELEM(*n_acute_a, double, 2, 0) = n_z;
//		// Solution b
//		n_x = s13 - eps12 * sqrt(Ms22);
//		n_y = s23 - sqrt(Ms11);
//		CV_MAT_ELEM(*n_acute_b, double, 0, 0) = n_x;
//		CV_MAT_ELEM(*n_acute_b, double, 1, 0) = n_y;
//		CV_MAT_ELEM(*n_acute_b, double, 2, 0) = n_z;
		
//		// 9. Get the translation vector in reference frame
//		double t_x, t_y, t_z, factor_1, factor_2;
//		// Solution a
//		factor_1 = cvNorm(n_acute_a) / (2 * s33);
//		factor_2 = te_norm_squared / (2 * cvNorm(n_acute_a));
//		t_x = factor_1 * (s13 - eps12 * sqrt(Ms22)) - factor_2 * (s13 + eps12 * sqrt(Ms22));
//		t_y = factor_1 * (s23 - sqrt(Ms11)) - factor_2 * (s23 + sqrt(Ms11));
//		t_z = factor_1 * s33 - factor_2 * s33;
//		CV_MAT_ELEM(*t_ref_a, double, 0, 0) = t_x;
//		CV_MAT_ELEM(*t_ref_a, double, 1, 0) = t_y;
//		CV_MAT_ELEM(*t_ref_a, double, 2, 0) = t_z;
//		// Solution b
//		factor_1 = cvNorm(n_acute_b) / (2 * s33);
//		factor_2 = te_norm_squared / (2 * cvNorm(n_acute_b));
//		t_x = factor_1 * (s13 + eps12 * sqrt(Ms22)) - factor_2 * (s13 - eps12 * sqrt(Ms22));
//		t_y = factor_1 * (s23 + sqrt(Ms11)) - factor_2 * (s23 - sqrt(Ms11));
//		t_z = factor_1 * s33 - factor_2 * s33;
//		CV_MAT_ELEM(*t_ref_b, double, 0, 0) = t_x;
//		CV_MAT_ELEM(*t_ref_b, double, 1, 0) = t_y;
//		CV_MAT_ELEM(*t_ref_b, double, 2, 0) = t_z;

//		cvScale(n_acute_a, n_a, 1/cvNorm(n_acute_a));
//		cvScale(n_acute_b, n_b, 1/cvNorm(n_acute_b));
//		// Solution a
//		cvGEMM(t_ref_a, n_a, -2 / v, I, 1, R_a, CV_GEMM_B_T);
//		cvGEMM(H, R_a, 1, 0, 0, R_a);
//		cvGEMM(R_a, t_ref_a, 1, 0, 0, t_a);
//		// Solution b
//		cvGEMM(t_ref_b, n_b, -2 / v, I, 1, R_b, CV_GEMM_B_T);
//		cvGEMM(H, R_b, 1, 0, 0, R_b);
//		cvGEMM(R_b, t_ref_b, 1, 0, 0, t_b);

//		r11 = CV_MAT_ELEM(*R_b, double, 0, 0);
//		r21 = CV_MAT_ELEM(*R_b, double, 1, 0);
//		r31 = CV_MAT_ELEM(*R_b, double, 2, 0);
//		r12 = CV_MAT_ELEM(*R_b, double, 0, 1);
//		r22 = CV_MAT_ELEM(*R_b, double, 1, 1);
//		r32 = CV_MAT_ELEM(*R_b, double, 2, 1);
//		r13 = CV_MAT_ELEM(*R_b, double, 0, 2);
//		r23 = CV_MAT_ELEM(*R_b, double, 1, 2);
//		r33 = CV_MAT_ELEM(*R_b, double, 2, 2);
//		tx = CV_MAT_ELEM(*t_b, double, 0, 0);
//		ty = CV_MAT_ELEM(*t_b, double, 1, 0);
//		tz = CV_MAT_ELEM(*t_b, double, 2, 0);
//	}

//	tx = ty = tz = 0;

//	//fstream f; f.open("TransTest.txt", ios::app);
//	//f << tx << "," << ty << "," << tz << "\r\n";
//	//f.close();
//	//cout << "" << tx << "," << ty << "," << tz << "\r\n";
//	// Orthogonality check
//	//std::cout << "Dot product n . o " << (r11*r12 + r21*r22 + r31*r32) << "\r\n";
//	//std::cout << "Dot product n . a " << (r11*r13 + r21*r23 + r31*r33) << "\r\n";
//	//std::cout << "Dot product o . a " << (r12*r13 + r22*r23 + r32*r33) << "\r\n";
//	//std::cout << "Magnitude of n " << sqrt(r11*r11 + r21*r21 + r31*r31) << "\r\n";
//	//std::cout << "Magnitude of o " << sqrt(r12*r12 + r22*r22 + r32*r32) << "\r\n";
//	//std::cout << "Magnitude of a " << sqrt(r13*r13 + r23*r23 + r33*r33) << "\r\n";

//	cvReleaseMat(&H_hat);
//	cvReleaseMat(&K_inv);
//	cvReleaseMat(&M);
//	cvReleaseMat(&S);
//	cvReleaseMat(&I);
//	cvReleaseMat(&H);
//	cvReleaseMat(&n_acute_a);
//	cvReleaseMat(&n_acute_b);
//	cvReleaseMat(&t_ref_a);
//	cvReleaseMat(&t_ref_b);
//	cvReleaseMat(&t_a);
//	cvReleaseMat(&t_b);

//	CvMat *W_f = cvCreateMat(4, 4, CV_64F);
//	CV_MAT_ELEM(*W_f, double, 0, 0) = r11;
//	CV_MAT_ELEM(*W_f, double, 1, 0) = r21;
//	CV_MAT_ELEM(*W_f, double, 2, 0) = r31;
//	CV_MAT_ELEM(*W_f, double, 3, 0) = 0;
//	CV_MAT_ELEM(*W_f, double, 0, 1) = r12;
//	CV_MAT_ELEM(*W_f, double, 1, 1) = r22;
//	CV_MAT_ELEM(*W_f, double, 2, 1) = r32;
//	CV_MAT_ELEM(*W_f, double, 3, 1) = 0;
//	CV_MAT_ELEM(*W_f, double, 0, 2) = r13;
//	CV_MAT_ELEM(*W_f, double, 1, 2) = r23;
//	CV_MAT_ELEM(*W_f, double, 2, 2) = r33;
//	CV_MAT_ELEM(*W_f, double, 3, 2) = 0;
//	CV_MAT_ELEM(*W_f, double, 0, 3) = -tx;
//	CV_MAT_ELEM(*W_f, double, 1, 3) = -ty;
//	CV_MAT_ELEM(*W_f, double, 2, 3) = -tz;
//	CV_MAT_ELEM(*W_f, double, 3, 3) = 1;

//	//cout << "rx1=" << r11 << "," << r21 << "," << r31;
//	//cout << "\nrx2=" << r12 << "," << r22 << "," << r32;
//	//cout << "\nrx3=" << r13 << "," << r23 << "," << r33 << "\n\n";

//	Mat transformResult = W_f;
//	transformResult.copyTo(transform);

//	cvReleaseMat(&W_f);
}

void HomographyDecomposer::decomposeIterative()
{
}

void HomographyDecomposer::decomposeRotational()
{
	// Normalize.. This seems to produce only the positive solution
    homography /= homography.at<double>(2, 2);
	
	double h11, h12, h13, h21, h22, h23, h31, h32, h33;
    h11 = homography.at<double>(0, 0);
    h21 = homography.at<double>(1, 0);
    h31 = homography.at<double>(2, 0);
    h12 = homography.at<double>(0, 1);
    h22 = homography.at<double>(1, 1);
    h32 = homography.at<double>(2, 1);
    h13 = homography.at<double>(0, 2);
    h23 = homography.at<double>(1, 2);
    h33 = homography.at<double>(2, 2);

    double fx, fy, cx, cy;
    fx = cameraIntrinsicParameters.at<double>(0, 0);
    fy = cameraIntrinsicParameters.at<double>(1, 1);
    cx = cameraIntrinsicParameters.at<double>(0, 2);
    cy = cameraIntrinsicParameters.at<double>(1, 2);

	double r11, r12, r13, r21, r22, r23, r31, r32, r33;
	double tx, ty, tz;

	r31 = h31;
	r11 = (h11 - cx * r31) / fx;
	r21 = (h21 - cy * r31) / fy;
	double norm_r1 = sqrt(pow(r31, 2) + pow(r21, 2) + pow(r11, 2));
	r11 /= norm_r1; r21 /= norm_r1; r31 /= norm_r1;
	
	r32 = h32;
	r12 = (h12 - cx * r32) / fx;
	r22 = (h22 - cy * r32) / fy;
	double norm_r2 = sqrt(pow(r32, 2) + pow(r22, 2) + pow(r12, 2));
	r12 /= norm_r2; r22 /= norm_r2; r32 /= norm_r2;

    //qDebug() << "Norm Diff: " << (norm_r1 - norm_r2);
    rotationalNormal = (norm_r1 + norm_r2) / 2.0;

	Point3f Rx = Point3f(r11, r21, r31);
	Point3f Ry = Point3f(r12, r22, r32);
	Point3f Rz = normalizedCrossProduct(Rx, Ry);
    //qDebug() << "Dot Product: " << (Rx.x * Ry.x + Rx.y * Ry.y + Rx.z * Ry.z);
    decompositionError = Rx.x * Ry.x + Rx.y * Ry.y + Rx.z * Ry.z;

	r13 = Rz.x;
	r23 = Rz.y;
	r33 = Rz.z;

	tx = (h13 - cx) / fx;
	ty = (h23 - cy) / fy;
	tz = h33;
    tx /= rotationalNormal; ty /= rotationalNormal; tz /= rotationalNormal;

    //qDebug() << "t: " << tx << ", " << ty << ", " << tz;
    //tx = ty = tz = 0;

    cv::Mat rotation = cv::Mat(3, 3, CV_64F);
    rotation.at<double>(0, 0) = r11;
    rotation.at<double>(1, 0) = r21;
    rotation.at<double>(2, 0) = r31;
    rotation.at<double>(0, 1) = r12;
    rotation.at<double>(1, 1) = r22;
    rotation.at<double>(2, 1) = r32;
    rotation.at<double>(0, 2) = r13;
    rotation.at<double>(1, 2) = r23;
    rotation.at<double>(2, 2) = r33;

    // Apply SVD to improve result
    //cv::SVD transformSVD(rotation, cv::SVD::FULL_UV);
    //cv::gemm(transformSVD.u, transformSVD.vt, 1.0, cv::Mat(), 0, rotation);

    for (int i = 0; i < 9; ++i)
        transform.at<double>(i / 3, i % 3) = rotation.at<double>(i / 3, i % 3);
    transform.at<double>(0, 3) = tx;
    transform.at<double>(1, 3) = ty;
    transform.at<double>(2, 3) = tz;
    transform.at<double>(3, 0) = 0;
    transform.at<double>(3, 1) = 0;
    transform.at<double>(3, 2) = 0;
    transform.at<double>(3, 3) = 1;

//    transform.at<double>(0, 0) = r11;
//    transform.at<double>(0, 1) = r12;
//    transform.at<double>(0, 2) = r13;
//    transform.at<double>(0, 3) = tx;
//    transform.at<double>(1, 0) = r21;
//    transform.at<double>(1, 1) = r22;
//    transform.at<double>(1, 2) = r23;
//    transform.at<double>(1, 3) = ty;
//    transform.at<double>(2, 0) = r31;
//    transform.at<double>(2, 1) = r32;
//    transform.at<double>(2, 2) = r33;
//    transform.at<double>(2, 3) = tz;
//    transform.at<double>(3, 0) = 0;
//    transform.at<double>(3, 1) = 0;
//    transform.at<double>(3, 2) = 0;
//    transform.at<double>(3, 3) = 1;

//    fstream debugFile; debugFile.open("Decomposition.txt", ios::app);
//    debugFile << "rotX," << r11 << "," << r21 << "," << r31 << ",";
//    debugFile << "rotY," << r12 << "," << r22 << "," << r32 << ",";
//    debugFile << "rotZ," << r13 << "," << r23 << "," << r33 << ",";
//    debugFile << "trans," << tx << "," << ty << "," << tz << ",";
//    debugFile << endl;
//    debugFile.close();
}

Point3f HomographyDecomposer::normalizedCrossProduct(Point3f X, Point3f Y)
{
	Point3f Z;

	Z.x = X.y * Y.z - X.z * Y.y;
	Z.y = X.z * Y.x - X.x * Y.z;
	Z.z = X.x * Y.y - X.y * Y.x;
	
	// Normalize and scale to magnitude
	float current_magnitude = sqrt(pow(Z.x, 2) + pow(Z.y, 2) + pow(Z.z, 2));
	Z.x = Z.x / current_magnitude;
	Z.y = Z.y / current_magnitude;
	Z.z = Z.z / current_magnitude;

	return Z;
}
