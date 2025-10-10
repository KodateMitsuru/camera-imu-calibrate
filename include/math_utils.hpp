#include <fftw3.h>

#include <Eigen/Dense>
#include <vector>

// Compute full (linear) cross-correlation a (*) b using FFTW.
// Returns vector of length (a.size() + b.size() - 1), same semantics as
// numpy.correlate(..., mode='full').
inline Eigen::VectorXd correlate_full(const Eigen::VectorXd& a,
                                      const Eigen::VectorXd& b) {
  const int len_a = static_cast<int>(a.size());
  const int len_b = static_cast<int>(b.size());
  if (len_a == 0 || len_b == 0) return Eigen::VectorXd();

  const int len_conv = len_a + len_b - 1;
  // fft length: next power-of-two could be used for performance; use len_conv
  // directly
  const int n = len_conv;

  // real-to-complex output size
  const int nc = n / 2 + 1;

  // allocate aligned buffers
  std::vector<double> A(n, 0.0);
  std::vector<double> B(n, 0.0);

  // copy inputs: a and reversed b for cross-correlation
  for (int i = 0; i < len_a; ++i) A[i] = a(i);
  // b needs to be reversed
  for (int i = 0; i < len_b; ++i) B[i] = b(len_b - 1 - i);

  // allocate complex spectra
  std::vector<fftw_complex> FA(nc);
  std::vector<fftw_complex> FB(nc);

  // plans
  fftw_plan planA = fftw_plan_dft_r2c_1d(n, A.data(), FA.data(), FFTW_ESTIMATE);
  fftw_plan planB = fftw_plan_dft_r2c_1d(n, B.data(), FB.data(), FFTW_ESTIMATE);

  fftw_execute(planA);
  fftw_execute(planB);

  // point-wise multiplication: FA * FB (complex multiply)
  std::vector<fftw_complex> Fprod(nc);
  for (int k = 0; k < nc; ++k) {
    double a_re = FA[k][0];
    double a_im = FA[k][1];
    double b_re = FB[k][0];
    double b_im = FB[k][1];
    // (a_re + i a_im) * (b_re + i b_im)
    Fprod[k][0] = a_re * b_re - a_im * b_im;
    Fprod[k][1] = a_re * b_im + a_im * b_re;
  }

  // inverse transform
  std::vector<double> conv(n, 0.0);
  fftw_plan planInv =
      fftw_plan_dft_c2r_1d(n, Fprod.data(), conv.data(), FFTW_ESTIMATE);
  fftw_execute(planInv);

  // cleanup plans
  fftw_destroy_plan(planA);
  fftw_destroy_plan(planB);
  fftw_destroy_plan(planInv);

  // normalize inverse by n (FFTW does not normalize)
  Eigen::VectorXd result(len_conv);
  for (int i = 0; i < len_conv; ++i)
    result(i) = conv[i] / static_cast<double>(n);

  return result;
}