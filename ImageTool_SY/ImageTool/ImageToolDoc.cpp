
// ImageToolDoc.cpp: CImageToolDoc 클래스의 구현
//

#include "pch.h"
#include "framework.h"
// SHARED_HANDLERS는 미리 보기, 축소판 그림 및 검색 필터 처리기를 구현하는 ATL 프로젝트에서 정의할 수 있으며
// 해당 프로젝트와 문서 코드를 공유하도록 해 줍니다.
#ifndef SHARED_HANDLERS
#include "ImageTool.h"
#endif

#include "ImageToolDoc.h"

#include <propkey.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include "IppImage.h"
#include "IppConvert.h"
#include "IppImage/IppEnhance.h"
#include "IppImage/IppFilter.h"
#include "IppImage/IppGeometry.h"
#include "CGammaCorrectionDlg.h"
#include "CHistogramDlg.h"
#include "CGaussianDlg.h"
#include "CAddNoiseDlg.h"
#include "CDiffusionDlg.h"
#include "CTranslateDlg.h"
#include "CResizeDlg.h"
#include "IppImage/IppFeature.h"
#include "CCannyEdgeDlg.h"
#include <algorithm>
#include <functional>
#include "CHarrisCornerDlg.h"

// CImageToolDoc

IMPLEMENT_DYNCREATE(CImageToolDoc, CDocument)

BEGIN_MESSAGE_MAP(CImageToolDoc, CDocument)
	ON_COMMAND(ID_IMAGE_INVERSE, &CImageToolDoc::OnImageInverse)
	ON_COMMAND(ID_BRIGHTNESS_CONTRAST, &CImageToolDoc::OnBrightnessContrast)
	ON_COMMAND(ID_GAMMA_CORRECTION, &CImageToolDoc::OnGammaCorrection)
	ON_COMMAND(ID_VIEW_HISTOGRAM, &CImageToolDoc::OnViewHistogram)
	ON_COMMAND(ID_HISTO_STRETCHING, &CImageToolDoc::OnHistoStretching)
	ON_COMMAND(ID_HISTO_EQUALIZATION, &CImageToolDoc::OnHistoEqualization)
	ON_COMMAND(ID_FILTER_MEAN, &CImageToolDoc::OnFilterMean)
	ON_COMMAND(ID_FILTER_WEIGHTED_MEAN, &CImageToolDoc::OnFilterWeightedMean)
	ON_COMMAND(ID_FILTER_MEAN2, &CImageToolDoc::OnFilterMean2)
	ON_COMMAND(ID_FILTER_WEIGHTED_MEAN2, &CImageToolDoc::OnFilterWeightedMean2)
	ON_COMMAND(ID_FILTER_GAUSSIAN, &CImageToolDoc::OnFilterGaussian)
	ON_COMMAND(ID_FILTER_LAPLACIAN, &CImageToolDoc::OnFilterLaplacian)
	ON_COMMAND(ID_FILTER_UNSHARP_MASK, &CImageToolDoc::OnFilterUnsharpMask)
	ON_COMMAND(ID_FILTER_HIGHBOOST, &CImageToolDoc::OnFilterHighboost)
	ON_COMMAND(ID_ADD_NOISE, &CImageToolDoc::OnAddNoise)
	ON_COMMAND(ID_FILTER_MEDIAN, &CImageToolDoc::OnFilterMedian)
	ON_COMMAND(ID_FILTER_DIFFUSION, &CImageToolDoc::OnFilterDiffusion)
	ON_COMMAND(ID_IMAGE_TRANSLATION, &CImageToolDoc::OnImageTranslation)
	ON_COMMAND(ID_IMAGE_RESIZE, &CImageToolDoc::OnImageResize)
	ON_COMMAND(ID_IMAGE_ROTATE90, &CImageToolDoc::OnImageRotate90)
	ON_COMMAND(ID_IMAGE_ROTATE180, &CImageToolDoc::OnImageRotate180)
	ON_COMMAND(ID_IMAGE_ROTATE270, &CImageToolDoc::OnImageRotate270)
	ON_COMMAND(ID_IMAGE_MIRROR, &CImageToolDoc::OnImageMirror)
	ON_COMMAND(ID_IMAGE_FLIP, &CImageToolDoc::OnImageFlip)
	ON_COMMAND(ID_EDGE_ROBERTS, &CImageToolDoc::OnEdgeRoberts)
	ON_COMMAND(ID_EDGE_PREWITT, &CImageToolDoc::OnEdgePrewitt)
	ON_COMMAND(ID_EDGE_SOBEL, &CImageToolDoc::OnEdgeSobel)
	ON_COMMAND(ID_EDGE_CANNY, &CImageToolDoc::OnEdgeCanny)
	ON_COMMAND(ID_HOUGH_LINE, &CImageToolDoc::OnHoughLine)
	ON_COMMAND(ID_HARRIS_CORNER, &CImageToolDoc::OnHarrisCorner)
END_MESSAGE_MAP()


// CImageToolDoc 생성/소멸

CImageToolDoc::CImageToolDoc() noexcept
{
	// TODO: 여기에 일회성 생성 코드를 추가합니다.

}

CImageToolDoc::~CImageToolDoc()
{
}

BOOL CImageToolDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: 여기에 재초기화 코드를 추가합니다.
	// SDI 문서는 이 문서를 다시 사용합니다.

	BOOL ret = TRUE;

	m_Dib = *(theApp.m_pNewDib);
	theApp.m_pNewDib = NULL;

	return TRUE;
}




// CImageToolDoc serialization

void CImageToolDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: 여기에 저장 코드를 추가합니다.
	}
	else
	{
		// TODO: 여기에 로딩 코드를 추가합니다.
	}
}

#ifdef SHARED_HANDLERS

// 축소판 그림을 지원합니다.
void CImageToolDoc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	// 문서의 데이터를 그리려면 이 코드를 수정하십시오.
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// 검색 처리기를 지원합니다.
void CImageToolDoc::InitializeSearchContent()
{
	CString strSearchContent;
	// 문서의 데이터에서 검색 콘텐츠를 설정합니다.
	// 콘텐츠 부분은 ";"로 구분되어야 합니다.

	// 예: strSearchContent = _T("point;rectangle;circle;ole object;");
	SetSearchContent(strSearchContent);
}

void CImageToolDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = nullptr;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != nullptr)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CImageToolDoc 진단

#ifdef _DEBUG
void CImageToolDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CImageToolDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CImageToolDoc 명령


BOOL CImageToolDoc::OnOpenDocument(LPCTSTR lpszPathName)
{
	if (!CDocument::OnOpenDocument(lpszPathName))
		return FALSE;

	// TODO:  여기에 특수화된 작성 코드를 추가합니다.

	return m_Dib.Load(CT2A(lpszPathName));
}


BOOL CImageToolDoc::OnSaveDocument(LPCTSTR lpszPathName)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.

	return m_Dib.Save(CT2A(lpszPathName));
}


void CImageToolDoc::OnImageInverse()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	IppByteImage img;
	IppDibToImage(m_Dib, img);

	IppInverse(img);

	IppDib dib;
	IppImageToDib(img, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnBrightnessContrast()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	IppByteImage img;
	IppDibToImage(m_Dib, img);

	IppBrightness(img, 60);
	IppContrast(img, 40);

	IppDib dib;
	IppImageToDib(img, dib);

	AfxNewBitmap(dib);

}


void CImageToolDoc::OnGammaCorrection()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CGammaCorrectionDlg dlg;
	if (dlg.DoModal() == IDOK) {

		IppByteImage img;
		IppDibToImage(m_Dib, img);

		IppGammaCorrection(img, dlg.m_fGamma);

		IppDib dib;
		IppImageToDib(img, dib);

		AfxNewBitmap(dib);

	}

}


void CImageToolDoc::OnViewHistogram()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CHistogramDlg dlg;
	dlg.SetImage(&m_Dib);
	dlg.DoModal();
}


void CImageToolDoc::OnHistoStretching()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	IppByteImage img;
	IppDibToImage(m_Dib, img);

	IppHistogramStretching(img);

	IppDib dib;
	IppImageToDib(img, dib);

	AfxNewBitmap(dib);

}


void CImageToolDoc::OnHistoEqualization()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	IppByteImage img;
	IppDibToImage(m_Dib, img);

	IppHistogramEqualizaiton(img);

	IppDib dib;
	IppImageToDib(img, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnFilterMean()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppFilterMean(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);

}


void CImageToolDoc::OnFilterWeightedMean()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppFilterWeightedMean(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnFilterMean2()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppFilterMean2(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnFilterWeightedMean2()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppFilterWeightedMean2(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnFilterGaussian()
{
	CGaussianDlg dlg;

	if (dlg.DoModal() == IDOK) {
		IppByteImage imgSrc;
		IppFloatImage imgDst;

		IppDibToImage(m_Dib, imgSrc);

		IppFilterGaussian(imgSrc, imgDst, dlg.m_fSigma);

		IppDib dib;
		IppImageToDib(imgDst, dib);

		AfxNewBitmap(dib);
	}
}


void CImageToolDoc::OnFilterLaplacian()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppFilterLaplacian(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnFilterUnsharpMask()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppFilterUnsharpMask(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnFilterHighboost()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	float alpha = 1.2f;

	IppFilterHighboost(imgSrc, imgDst, alpha);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnAddNoise()
{
	CAddNoiseDlg dlg;
	if (dlg.DoModal() == IDOK) {
		IppByteImage imgSrc;
		IppByteImage imgDst;

		IppDibToImage(m_Dib, imgSrc);

		if (dlg.m_nNoiseType == 0)
			IppNoiseGaussian(imgSrc, imgDst, dlg.m_nAmount);
		else
			IppNoiseSaltNPepper(imgSrc, imgDst, dlg.m_nAmount);

		IppDib dib;
		IppImageToDib(imgDst, dib);

		AfxNewBitmap(dib);
	}

}


void CImageToolDoc::OnFilterMedian()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppFilterMedian(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnFilterDiffusion()
{
	CDiffusionDlg dlg;

	if (dlg.DoModal() == IDOK) {

		IppByteImage imgSrc;
		IppFloatImage imgDst;

		IppDibToImage(m_Dib, imgSrc);

		IppFilterDiffusion(imgSrc, imgDst, dlg.m_fLambda, dlg.m_fK, dlg.m_nIteration);

		IppDib dib;
		IppImageToDib(imgDst, dib);

		AfxNewBitmap(dib);
	}
}


void CImageToolDoc::OnImageTranslation()
{
	CTranslateDlg dlg;

	if (dlg.DoModal() == IDOK) {

		IppByteImage imgSrc;
		IppByteImage imgDst;

		IppDibToImage(m_Dib, imgSrc);

		IppTranslate(imgSrc, imgDst, dlg.m_nNewSX, dlg.m_nNewSY);

		IppDib dib;
		IppImageToDib(imgDst, dib);

		AfxNewBitmap(dib);
	}
}


void CImageToolDoc::OnImageResize()
{
	CResizeDlg dlg;
	dlg.m_nOldWidth = m_Dib.GetWidth();
	dlg.m_nOldHeight = m_Dib.GetHeight();
	if (dlg.DoModal() == IDOK) {
		IppByteImage imgSrc;
		IppByteImage imgDst;

		IppDibToImage(m_Dib, imgSrc);

		switch (dlg.m_ninterpolation) {
		case 0: 
			IppResizeNearest(imgSrc, imgDst, dlg.m_nNewWidth, dlg.m_nNewHeight);
			break;
		case 1:
			IppResizeBilinear(imgSrc, imgDst, dlg.m_nNewWidth, dlg.m_nNewHeight);
			break;
		case 2:
			IppResizeCubic(imgSrc, imgDst, dlg.m_nNewWidth, dlg.m_nNewHeight);
			break;
		}
		IppDib dib;
		IppImageToDib(imgDst, dib);

		AfxNewBitmap(dib);
	}
}


void CImageToolDoc::OnImageRotate90()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppRotate90(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnImageRotate180()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppRotate180(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnImageRotate270()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppRotate270(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnImageMirror()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppMirror(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnImageFlip()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppFlip(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnEdgeRoberts()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppEdgeRoberts(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnEdgePrewitt()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppEdgePrewitt(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnEdgeSobel()
{
	IppByteImage imgSrc;
	IppByteImage imgDst;

	IppDibToImage(m_Dib, imgSrc);

	IppEdgeSobel(imgSrc, imgDst);

	IppDib dib;
	IppImageToDib(imgDst, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnEdgeCanny()
{
	CCannyEdgeDlg dlg;
	if (dlg.DoModal() == IDOK) {
		IppByteImage imgSrc;
		IppByteImage imgDst;

		IppDibToImage(m_Dib, imgSrc);

		IppEdgeCanny(imgSrc, imgDst, dlg.m_fSigma, dlg.m_fLowTh, dlg.m_fHighTh);
		
		IppDib dib;
		IppImageToDib(imgDst, dib);
		
		AfxNewBitmap(dib);
	}

}


void CImageToolDoc::OnHoughLine()
{
	IppByteImage imgSrc;
	IppByteImage imgEdge;

	IppDibToImage(m_Dib, imgSrc);

	IppEdgeCanny(imgSrc, imgEdge, 1.4f, 30.f, 60.f);

	std::vector<IppLineParam> lines;
	IppHoughLine(imgEdge, lines);

	if (lines.size() == 0) {
		AfxMessageBox(_T("검출된 직선이 없습니다. "));
		return;
	}

	std::sort(lines.begin(), lines.end());

	//최대 10개의 직선만 화면에 그려줌.
	int cnt = __min(10, lines.size());
	for (int i = 0; i < cnt; i++)
		IppDrawLine(imgSrc, lines[i], 255);

	IppDib dib;
	IppImageToDib(imgSrc, dib);

	AfxNewBitmap(dib);
}


void CImageToolDoc::OnHarrisCorner()
{
	CHarrisCornerDlg dlg;
	if (dlg.DoModal() == IDOK) {
		IppByteImage img;
		IppDibToImage(m_Dib, img);
		std::vector<IppPoint> corners;

		IppHarrisCorner(img, corners, dlg.m_nHarrisTh);

		BYTE** ptr = img.GetPixels2D();

		int x, y;
		for (IppPoint cp : corners) {
			x = cp.x;
			y = cp.y;

			ptr[y - 1][x - 1] = ptr[y - 1][x] = ptr[y - 1][x + 1] = 0;
			ptr[y][x - 1] = ptr[y][x] = ptr[y][x + 1] = 0;
			ptr[y + 1][x - 1] = ptr[y + 1][x] = ptr[y + 1][x + 1] = 0;
		}
		IppDib dib;
		IppImageToDib(img, dib);

		AfxNewBitmap(dib);
	}
}
