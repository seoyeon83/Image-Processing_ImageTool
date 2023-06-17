#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <random>
#include <algorithm>
#include "IppFilter.h"

void IppFilterMean(IppByteImage& imgSrc, IppByteImage& imgDst) {

	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst = imgSrc;

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int mask[3][3] = {
		{1,1,1},
		{1,1,1},
		{1,1,1}
	};

	int i, j, m, n, sum;
	for (j = 1; j < h - 1; j++) {
		for (i = 1; i < w - 1; i++) {
			sum = 0;
			for(m = 0; m < 3; m++)
				for (n = 0; n < 3; n++) {
					sum += (pSrc[j - 1 + m][i - 1 + n] * mask[m][n]);
				}

			pDst[j][i] = static_cast<BYTE>(limit(sum / 9. + 0.5));
		}
	}
}

void IppFilterWeightedMean(IppByteImage& imgSrc, IppByteImage& imgDst) {
	
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst = imgSrc;

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int mask[3][3] = {
		{1,2,1},
		{2,4,2},
		{1,2,1}
	};

	int i, j, m, n, sum;
	for (j = 1; j < h - 1; j++) {
		for (i = 1; i < w - 1; i++) {
			sum = 0;
			for (m = 0; m < 3; m++)
				for (n = 0; n < 3; n++) {
					sum += (pSrc[j - 1 + m][i - 1 + n] * mask[m][n]);
				}

			pDst[j][i] = static_cast<BYTE>(limit(sum / 16. + 0.5));
		}
	}
}

void IppFilterMean2(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst = imgSrc;

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int mask[5][5] = {
		{1,1,1,1,1},
		{1,1,1,1,1},
		{1,1,1,1,1},
		{1,1,1,1,1},
		{1,1,1,1,1}
	};
	int i, j, m, n, sum;

	for (j = 2; j < h - 2; j++)
		for (i = 2; i < w - 2; i++) {
			sum = 0;
			for (m = 0; m < 5; m++)
				for (n = 0; n < 5; n++) {
					sum += (pSrc[j - 2 + m][i - 2 + n] * mask[m][n]);
				}

			pDst[j][i] = static_cast<BYTE>(limit(sum / 25. + 0.5));
		}
}


void IppFilterWeightedMean2(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst = imgSrc;

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int mask[5][5] = {
		{1,4,6,4,1},
		{4,16,24,16,4},
		{6,24,36,24,6},
		{4,16,24,16,4},
		{1,4,6,4,1}
	};

	int i, j, m, n, sum;

	for (j = 2; j < h - 2; j++)
		for (i = 2; i < w - 2; i++) {
			sum = 0;
			for (m = 0; m < 5; m++)
				for (n = 0; n < 5; n++) {
					sum += (pSrc[j - 2 + m][i - 2 + n] * mask[m][n]);
				}

			pDst[j][i] = static_cast<BYTE>(limit(sum / 256. + 0.5));
		}
}


const float PI_F = 3.1415926535;
void IppFilterGaussian(IppByteImage& imgSrc, IppFloatImage& imgDst, float sigma) {

	register int i, j, k, x;

	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst.CreateImage(w, h);

	BYTE** pSrc = imgSrc.GetPixels2D();
	float** pDst = imgDst.GetPixels2D();

	// 1차원 가우시안 마스크 & 실수 연산을 위한 버퍼 이미지 생성하는 부분
	int dim = static_cast<int>(2 * 4 * sigma + 1.0);
	if (dim < 3) dim = 3;
	if (dim % 2 == 0) dim++;
	int dim2 = dim / 2;

	IppFloatImage imgMask(dim, 1);
	float* pMask = imgMask.GetPixels();

	for (i = 0; i < dim; i++) {
		x = i - dim2;
		pMask[i] = exp(-(x * x) / (2 * sigma * sigma)) / (sqrt(2 * PI_F) * sigma);
	}

	IppFloatImage imgBuf(w, h);
	float** pBuf = imgBuf.GetPixels2D();

	//세로 방향 마스크 연산
	float sum1, sum2;
	for(i = 0; i < w; i++)
		for (j = 0; j < h; j++) {
			sum1 = sum2 = 0.f;

			for (k = 0; k < dim; k++) {
				x = k - dim2 + j;
				if (x >= 0 && x < h) {
					sum1 += pMask[k];
					sum2 += (pMask[k] * pSrc[x][i]);
				}
			}

			pBuf[j][i] = sum2 / sum1;
		}


	//가로방향 연산
	for(j = 0; j <h; j++)
		for (i = 0; i < w; i++) {
			sum1 = sum2 = 0.f;

			for (k = 0; k < dim; k++) {
				x = k - dim2 + i;
				if (x >= 0 && x < w) {
					sum1 += pMask[k];
					sum2 += (pMask[k] * pBuf[j][x]);
				}
			}

			pDst[j][i] = sum2 / sum1;
		}
}

// 라플라시안 필터링
void IppFilterLaplacian(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst = imgSrc;

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j, m, n, sum;
	for (j = 1; j < h - 1; j++) {
		for (i = 1; i < w - 1; i++) {
			// (왼쪽, 위쪽, 오른쪽, 아래쪽 (4방향) 픽셀 값의 합) - (4 * 나의 픽셀값)
			// 주변 4방향 픽셀과 나의 픽셀값의 차이
			sum = pSrc[j - 1][i] + pSrc[j][i - 1] + pSrc[j + 1][i] + pSrc[j][i + 1] - 4 * pSrc[j][i];
			
			pDst[j][i] = static_cast<BYTE>(limit(sum  + 128));  // 음수 표현을 위해 +128
		}
	}

}

// 라플라시안 필터를 이용한 언샤프 마스크
void IppFilterUnsharpMask(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst = imgSrc;

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	/*
	// 수식을 이용한 구현
	int i, j, m, n, sum;
	for (j = 1; j < h - 1; j++) {
		for (i = 1; i < w - 1; i++) {
			sum = 5 * pSrc[j][i] - pSrc[j - 1][i] - pSrc[j][i - 1] - pSrc[j + 1][i] - pSrc[j][i + 1];
			// 나의 픽셀값 * 5 - (상하좌우 픽셀 값의 합)

			pDst[j][i] = static_cast<BYTE>(limit(sum));
		}
	}
	*/

	// 공간적 필터링 연산으로 구현
	int mask[3][3] = {
		{0, -1, 0},
		{-1, 5, -1},
		{0, -1, 0}
	};

	int i, j, m, n, sum;
	for (j = 1; j < h - 1; j++) {     // 3x3 마스크이기 때문에 i = 1 ~ h-1
		for (i = 1; i < w - 1; i++) { // j도 마찬가지
			// 주변 4방향과 픽셀 값 차이 + 본인의 픽셀 값 계산
			sum = 0;
			
			for (m = 0; m < 3; m++)
				for (n = 0; n < 3; n++) {
					sum += (pSrc[j - 1 + m][i - 1 + n] * mask[m][n]);
				}
			pDst[j][i] = static_cast<BYTE>(limit(sum));
		}
	}
}

// 하이부스트 필터
void IppFilterHighboost(IppByteImage& imgSrc, IppByteImage& imgDst, float alpha)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst = imgSrc;

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j, m, n, sum;
	for (j = 1; j < h - 1; j++) {
		for (i = 1; i < w - 1; i++) {
			sum = (4 + alpha) * pSrc[j][i] - pSrc[j - 1][i] - pSrc[j][i - 1] - pSrc[j + 1][i] - pSrc[j][i + 1];

			// 알파 값에 따라 명암비 조정. 알파가 클 수록 명암비 커짐
			// (4 + 알파 값) * 나의 픽셀 값 - (상하좌우 픽셀 값의 합)

			// 결과가 실수 값일 수도 있기 때문에 0.5 더해 반올림

			pDst[j][i] = static_cast<BYTE>(limit(sum+0.5));
		}
	}
}


// 가우시안 잡음
void IppNoiseGaussian(IppByteImage& imgSrc, IppByteImage& imgDst, int amount)
{
	int size = imgSrc.GetSize();

	imgDst = imgSrc;
	BYTE* pDst = imgDst.GetPixels();

	// 시드(seed) 값 부여하여 매번 다른 난수 발생시키기
	unsigned int seed = static_cast<unsigned int>(time(NULL));
	std::default_random_engine generator(seed);
	std::normal_distribution<double> distribution(0.0, 1.0);

	double rn;
	for (int i = 0; i < size; i++) {
		rn = distribution(generator) * 255 * amount / 100;
		pDst[i] = static_cast<BYTE>(limit(pDst[i] + rn));
	}
}

// 소금&후추 잡음
void IppNoiseSaltNPepper(IppByteImage& imgSrc, IppByteImage& imgDst, int amount)
{
	int size = imgSrc.GetSize();

	imgDst = imgSrc;
	BYTE* pDst = imgDst.GetPixels();
	
	// 균일 분포 난수로 픽셀 좌표를 선택
	unsigned int seed = static_cast<unsigned int>(time(NULL));
	std::default_random_engine generator(seed);
	std::uniform_int_distribution<int> distribution(0, size - 1);

	int num = size * amount / 100;  // 잡음이 추가될 픽셀의 개수
	for (int i = 0; i < num; i++) {
		// 잡음이 추가될 위치는 난수로 지정,  i가 짝수면 0, i가 홀수면 255
		pDst[distribution(generator)] = (i & 0x01) * 255;
	}

}

// 미디언 필터
void IppFilterMedian(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst = imgSrc;

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j;
	BYTE m[9];
	for (j = 1; j < h - 1; j++) {
		for (i = 1; i < w - 1; i++) {
			// 마스크 별 픽셀값 구하기
			m[0] = pSrc[j - 1][i - 1]; m[1] = pSrc[j - 1][i]; m[2] = pSrc[j - 1][i + 1];
			m[3] = pSrc[j][i - 1]; m[4] = pSrc[j][i]; m[5] = pSrc[j][i + 1];
			m[6] = pSrc[j + 1][i - 1]; m[7] = pSrc[j + 1][i]; pSrc[j + 1][i + 1];

			std::sort(m, m + 9); // 주소를 입력으로 받아 오름차순으로 정렬
			pDst[j][i] = m[4]; // 중앙값
		}
	}
}

// 비등방성 확산 필터
void IppFilterDiffusion(IppByteImage& imgSrc, IppFloatImage& imgDst, float lambda, float k, int iter)
{   // lambda, k, iter(반복 횟수)를 입력 받음

	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	IppFloatImage imgCpy;
	imgCpy.Convert(imgSrc);

	imgDst = imgCpy;

	float** pCpy = imgCpy.GetPixels2D();
	float** pDst = imgDst.GetPixels2D();

	// iter횟수만큼 비등방성 확산 알고리즘 수행

	register int i, x, y;
	float gradn, grads, grade, gradw; // 방향 별 그래디언트(기울기)
	float gcn, gcs, gce, gcw;  // 방향 별 전달 계수 c
	float k2 = k * k;   // 상수 k

	for (i = 0; i < iter; i++) {
		for(y = 1; y <h-1; y++)
			for (x = 1; x < w - 1; x++) {
				// 변화량(그래디어트)
				gradn = pCpy[y - 1][x] - pCpy[y][x];
				grads = pCpy[y + 1][x] - pCpy[y][x];
				grade = pCpy[y][x-1] - pCpy[y][x];
				gradw = pCpy[y][x+1] - pCpy[y][x];

				// 전달 계수 함수 계산 결과
				gcn = gradn / (1.0f + gradn * gradn / k2);
				gcs = grads / (1.0f + grads * grads / k2);
				gce = grade / (1.0f + grade * grade / k2);
				gcw = gradw / (1.0f + gradw * gradw / k2);

				// 결과 저장
				pDst[y][x] = pCpy[y][x] + lambda * (gcn + gcs + gce + gcw);
			}

		// 반복 횟수가 남았다면, pDst에 저장된 결과를 pCpy에 복사. (결과가 누적되어 보일 수 있도록)
		if (i < iter - 1)
			memcpy(imgCpy.GetPixels(), imgDst.GetPixels(), sizeof(float) * w * h);
	}
}


