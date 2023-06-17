#include "IppGeometry.h"

// 영상의 이동 변환
void IppTranslate(IppByteImage& imgSrc, IppByteImage& imgDst, int sx, int sy)
{   // parameter: 원본 영상, 결과 영상, 이동할 x 값, 이동할 y 값

	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	// 원본 영상의 크기에 맞게 영상 생성
	imgDst.CreateImage(w, h);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	// i, j는 출력 영상, x, y는 입력 영상의 인덱스
	int i, j, x, y;
	for (j = 0; j < h; j++) {
		for (i = 0; i < w; i++) {
			// 위치(인덱스) 계산)
			x = i - sx;
			y = j - sy;

			// x, y가 영상 범위를 벗어나지 않은 경우 결과 적용
			if (x >= 0 && x < w && y >= 0 && y < h) {
				pDst[j][i] = pSrc[y][x];
			}
		}
	}
}

// 최근방 이웃 보간법
void IppResizeNearest(IppByteImage& imgSrc, IppByteImage& imgDst, int nw, int nh)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	// 출력 영상 생성
	imgDst.CreateImage(nw, nh);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	// 출력 영상 좌표: i, j / 입력 영상 좌표: x, y
	int i, j, x, y;
	double rx, ry; // 입력 영상에서 참조할 픽셀값의 실수 좌표

	for (j = 0; j < nh; j++) {
		for (i = 0; i < nw; i++) {
			// 픽셀값 계산
			rx = static_cast<double>(w - 1) * i / (nw - 1);
			ry = static_cast<double>(h - 1) * j / (nh - 1);

			x = static_cast<int>(rx + 0.5);  // 반올림
			y = static_cast<int>(ry + 0.5);

			// 입력 영상의 최대 크기를 넘어갈 경우 크기의 가장 마지막에 있는 위치로. (limit 처럼)
			if (x >= w) x = w - 1;
			if (y >= h) y = h - 1;

			pDst[j][i] = pSrc[y][x];
		}
	}
}

// 양선형 이웃 보간법
void IppResizeBilinear(IppByteImage& imgSrc, IppByteImage& imgDst, int nw, int nh)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst.CreateImage(nw, nh);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j;
	double rx, ry; // 입력 영상에서 참조할 픽셀값의 실수 좌표
	int x1, y1, x2, y2; // rx, ry를 둘러싼 4개의 픽셀 좌표
	double p, q, value; // 주변 픽셀과의 거리, 결과 값

	for (j = 0; j < nh; j++) {
		for (i = 0; i < nw; i++) {
			// 출력 영상의 픽셀 위치에 대한 입력 영상의 픽셀 위치 구하기
			rx = static_cast<double>(w - 1) * i / (nw - 1);
			ry = static_cast<double>(h - 1) * j / (nh - 1);

			// rx, ry의 주변 픽셀 좌표 구하기(i, j, i+1, j+1)
			x1 = static_cast<double>(rx);
			y1 = static_cast<double>(ry);
			x2 = x1 + 1; if (x2 == w) x2 = w - 1;
			y2 = y1 + 1; if (y2 == h) y2 = h - 1;

			p = rx - x1;
			q = ry - y1;

			// 픽셀 값 계산
			value = (1. - p) * (1. - q) * pSrc[y1][x1]
				+ p * (1. - q) * pSrc[y1][x2]
				+ (1. - p) * q * pSrc[y2][x1]
				+ p * q * pSrc[y2][x2];

			// 픽셀 값 반올림 후 저장
			pDst[j][i] = static_cast<BYTE>(limit(value + 0.5));
		}
	}// for
}

// 3차 회선 보간법(가중치 함수로 수식 계산)
double cubic_interpolation(double v1, double v2, double v3, double v4, double d)
{
	double v, p1, p2, p3, p4;

	p1 = 2 * v2;
	p2 = -v1 + v3;
	p3 = 2 * v1 - 5 * v2 + 4 * v3 - v4;
	p4 = -v1 + 3 * v2 - 3 * v3 + v4;

	v = (p1 + d * (p2 + d * (p3 + d * p4))) / 2.;

	return v;
}

// 3차 회선 보간법
void IppResizeCubic(IppByteImage& imgSrc, IppByteImage& imgDst, int nw, int nh)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst.CreateImage(nw, nh);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j, x1, x2, x3, x4, y1, y2, y3, y4;
	double v1, v2, v3, v4, v;
	double rx, ry, p, q;

	for (j = 0; j < nh; j++) {
		for (i = 0; i < nw; i++) {
			// 역방향으로 추적한 입력 영상의 픽셀 위치
			rx = static_cast<double>(w - 1) * i / (nw - 1);
			ry = static_cast<double>(h - 1) * j / (nh - 1);

			x2 = static_cast<int>(rx);
			x1 = x2 - 1; if (x1 < 0) x1 = 0;
			x3 = x2 + 1; if (x3 >= w) x3 = w - 1;
			x4 = x2 + 2; if (x4 >= w) x4 = w - 1;
			p = rx - x2;

			y2 = static_cast<int>(ry);
			y1 = y2 - 1; if (y1 < 0) y1 = 0;
			y3 = y2 + 1; if (y3 >= h) y3 = h - 1;
			y4 = y2 + 2; if (y4 >= h) y4 = h - 1;
			q = ry - y2;

			// 총 5번 보간 진행
			// 행 방향으로
			v1 = cubic_interpolation(pSrc[y1][x1], pSrc[y1][x2], pSrc[y1][x3], pSrc[y1][x4], p);
			v2 = cubic_interpolation(pSrc[y2][x1], pSrc[y2][x2], pSrc[y2][x3], pSrc[y2][x4], p);
			v3 = cubic_interpolation(pSrc[y3][x1], pSrc[y3][x2], pSrc[y3][x3], pSrc[y3][x4], p);
			v4 = cubic_interpolation(pSrc[y4][x1], pSrc[y4][x2], pSrc[y4][x3], pSrc[y4][x4], p);

			// 열 방향으로
			v = cubic_interpolation(v1, v2, v3, v4, q);
			// 결과 픽셀 값 반올림 하기
			pDst[j][i] = static_cast<BYTE>(limit(v + 0.5));
		}
	}
}

// 특수 각도(90도) 회전
void IppRotate90(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	// w, h 반대로
	imgDst.CreateImage(h, w);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j;
	for (j = 0; j < w; j++)
		for (i = 0; i < h; i++)
		{
			pDst[j][i] = pSrc[h - 1 - i][j];
		}
}

// 특수 각도(180도) 회전
void IppRotate180(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst.CreateImage(w, h);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j;
	for (j = 0; j < h; j++)
		for (i = 0; i < w; i++)
		{
			pDst[j][i] = pSrc[h - 1 - j][w - 1 - i];
		}
}

// 특수 각도(240도) 회전
void IppRotate270(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	IppByteImage cpy = imgSrc;

	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	// w, h 반대로
	imgDst.CreateImage(h, w);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j;
	for (j = 0; j < w; j++)
		for (i = 0; i < h; i++)
		{
			pDst[j][i] = pSrc[i][w - 1 - j];
		}
}

// 좌우대칭
void IppMirror(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst.CreateImage(w, h);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j;
	for (j = 0; j < h; j++)
		for (i = 0; i < w; i++)
		{
			pDst[j][i] = pSrc[j][w - 1 - i];
		}
}

// 상하대칭
void IppFlip(IppByteImage& imgSrc, IppByteImage& imgDst)
{
	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	imgDst.CreateImage(w, h);

	BYTE** pSrc = imgSrc.GetPixels2D();
	BYTE** pDst = imgDst.GetPixels2D();

	int i, j;
	for (j = 0; j < h; j++)
		for (i = 0; i < w; i++)
		{
			pDst[j][i] = pSrc[h - 1 - j][i];
		}
}



