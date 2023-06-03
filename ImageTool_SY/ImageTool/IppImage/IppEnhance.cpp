#include <math.h>
#include "IppEnhance.h"


void IppInverse(IppByteImage& img) {
	int size = img.GetSize();
	BYTE* p = img.GetPixels();

	for (int i = 0; i < size; i++) {
		p[i] = 255 - p[i];
	}
}


void IppBrightness(IppByteImage& img, int n) {
	int size = img.GetSize();
	BYTE* p = img.GetPixels();

	for (int i = 0; i < size; i++) {
		p[i] = limit(p[i] + n);
	}
}

void IppContrast(IppByteImage& img, int n) {
	int size = img.GetSize();
	BYTE* p = img.GetPixels();

	for(int i = 0; i < size; i++) {
		p[i] = static_cast<BYTE>(limit(p[i] + (p[i] - 128) * n / 100));
	}
}

void IppGammaCorrection(IppByteImage& img, float gamma)
{
	float inv_gamma = 1.f / gamma;

	float gamma_table[256];
	for (int i = 0; i < 256; i++) {
		gamma_table[i] = pow((i / 255.f), inv_gamma);
	}

	int size = img.GetSize();
	BYTE* p = img.GetPixels();

	for (int i = 0; i < size; i++) {
		p[i] = static_cast<BYTE>(limit(gamma_table[p[i]] * 255 + 0.5f));
	}
}

void IppHistogram(IppByteImage& img, float histo[256]) {

	int size = img.GetSize();
	BYTE* p = img.GetPixels();

	//히스토그램 계산
	int cnt[256];
	memset(cnt, 0, sizeof(int) * 256);
	for (int i = 0; i < size; i++) {
		cnt[p[i]]++;
	}

	for (int i = 0; i < 256; i++) {
		histo[i] = static_cast<float>(cnt[i]) / size;
	}

}



void IppHistogramStretching(IppByteImage& img) {
	int size = img.GetSize();
	BYTE* p = img.GetPixels();

	BYTE gray_max, gray_min;
	gray_max = gray_min = p[0];
	for (int i = 1; i < size; i++) {
		if (gray_max < p[i]) gray_max = p[i];
		if (gray_min > p[i]) gray_min = p[i];
	}

	for (int i = 0; i < size; i++) {
		p[i] = (p[i] - gray_min) * 255 / (gray_max - gray_min);
	}


}


void IppHistogramEqualizaiton(IppByteImage& img) {

	int size = img.GetSize();
	BYTE* p = img.GetPixels();

	float hist[256];
	IppHistogram(img, hist);

	float cdf[256] = { 0.0 };
	cdf[0] = hist[0];
	for (int i = 1; i < 256; i++) {
		cdf[i] = cdf[i - 1] + hist[i];
	}

	for (int i = 0; i < size; i++) {
		p[i] = static_cast<BYTE>(limit(cdf[p[i]] * 255));
	}
}
