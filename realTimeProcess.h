#pragma once
#pragma once
#include <QtWidgets/qinputdialog.h>
#include <QtWidgets/qformlayout.h>
#include <QtWidgets/QMainWindow>
#include "pclHeaders.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include "ui_realTimeProcess.h"
#include <QMessageBox>

#include <QFileDialog>
#include "logger.h"
#include <boost/thread/thread.hpp>
#include "registrations.h"
#include "postProcess.h"
#include "dialogs.h"
#include "matcher.h"
#include <QSet>
#include <QThread>
// ���߳�ͬ��
#include <mutex>
#include <condition_variable>
#include <atomic>


class realTimeProcess : public QMainWindow {
	Q_OBJECT

public:
	realTimeProcess(QWidget* parent = nullptr);
	~realTimeProcess();

private:
	QThread m_workerThread;
	// �Ի���
	QDialog* m_progressDialog = nullptr;
	vector<PointCloudT::Ptr> points;
	vector<PointCloudT::Ptr> pointsDown;
	mutex pointsMutex;
	condition_variable pointsCond;
	//��������
    PointCloudT::Ptr cloudShowDown;
	//�������
	std::atomic<bool> isRunning{false};
	int cloudNum = 0;
	mutex textMutex;

	Ui::realTimeProcessClass ui;

	PointCloudT::Ptr cloudShow;
	PointCloudT::Ptr cloudTar;
	PointCloudT::Ptr cloudSrc;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view;
	//��ر�־λ
    bool isLoadSrc = false;
    bool isLoadTar = false;
	bool isReadDone= false;
	//�༭��ǣ�0ΪԴ���ƣ�1ΪĿ�����
    int editFlag = 0;
	
	//ƥ��ģ��
	Matcher matcher;

	//��־
	Logger logger;

	void initialVtkWidget();
	//����ļ���
	QString path;
private slots:
	//todo:���ص����ļ�
	void loadSrc();
	void loadTar();
	//todo:����ʵʱƴ��ϵͳ
	void enterSystem();
	//todo:��ʼƴ��
	void startReg();
	//todo:��ʼ����
	void startPostProcess();
	//todo:�������
	void clearPointCloud();
	//todo:������ɫ�趨
    void setBackgroundColor();
	//todo:���浱ǰ����
	void saveCurrentPointCloud();
	//todo:errorInfo
    void errorInfo(int errNum);
	//todo:������ʾ����
	void updatePointCloudShow();

	//���ĵ�ǰ�༭Ŀ��
	void editSrc();
	void editTar();

	//�ϲ����Ƶ�Ŀ�����
    void mergeCloud();

	//ʵʱ��ά�ؽ�
	void realTimeSys();
	void folderMonitor();
	void stopSystem();
	void updateWhole();
	void showDialog();
	void PostProcessWhole();
signals:
	void updateVisualizationRequested();
};
