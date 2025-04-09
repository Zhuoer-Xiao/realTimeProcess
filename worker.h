// cloud_worker.h
#ifndef CLOUD_WORKER_H
#define CLOUD_WORKER_H

#include <QObject>
#include <QFileSystemWatcher>
#include <QFileInfo>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename PointT>
struct CloudWithMetadata {
    using Ptr = std::shared_ptr<CloudWithMetadata<PointT>>;
    typename pcl::PointCloud<PointT>::Ptr cloud;
    QString filePath;
    qint64 timestamp;
};

/**
 * @class CloudWorker
 * @brief ֧�ֶ��ʽ�ĵ����ļ��������ع����߳�
 *
 * �������ԣ�
 * - ʵʱ����ļ��б仯
 * - �Զ�ʶ�� PCD/PLY ��ʽ
 * - ��ʱ����ĵ������ݷ�װ
 * - �����ļ�ȥ�ش���
 */
template<typename PointT>
class CloudWorker : public QObject {
    Q_OBJECT
public:
    explicit CloudWorker(QObject* parent = nullptr)
        : QObject(parent), m_minFileInterval(500) // ��С�ļ�������500ms
    {
        connect(&m_watcher, &QFileSystemWatcher::fileChanged,
            this, &CloudWorker::handleFileChange);
    }

public slots:
    /// ���ü��Ŀ¼���Զ����������ļ�
    void setWatchDirectory(const QString& dir) {
        m_watcher.addPath(dir);
        processExistingFiles(dir);
    }

    /// ������С�ļ�����������������
    void setMinFileInterval(int ms) { m_minFileInterval = ms; }

signals:
    void cloudReady(typename CloudWithMetadata<PointT>::Ptr data);
    void errorOccurred(const QString& message);

private slots:
    void handleFileChange(const QString& path) {
        QFileInfo fi(path);
        QString ext = fi.suffix().toLower();

        // ��ʽ����
        if (ext != "pcd" && ext != "ply") return;

        // ȥ�ؼ�飺ͬһ�ļ�500ms�ڲ��ظ�����
        qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
        if (m_fileTimestamps.contains(path) &&
            (currentTime - m_fileTimestamps[path]) < m_minFileInterval) {
            return;
        }
        m_fileTimestamps[path] = currentTime;

        // �첽�����ֹ����
        QTimer::singleShot(0, this, [this, path, ext]() {
            processCloudFile(path, ext);
            });
    }

private:
    /// �����������ļ��ĺ��ķ���
    void processCloudFile(const QString& path, const QString& format) {
        typename CloudWithMetadata<PointT>::Ptr data(new CloudWithMetadata<PointT>);
        data->filePath = path;
        data->timestamp = QDateTime::currentMSecsSinceEpoch();
        data->cloud.reset(new pcl::PointCloud<PointT>);

        try {
            // ��֧����ͬ��ʽ
            if (format == "pcd") {
                if (pcl::io::loadPCDFile<PointT>(path.toStdString(), *data->cloud) == -1) {
                    throw std::runtime_error("Invalid PCD file structure");
                }
            }
            else if (format == "ply") {
                pcl::PLYReader reader;
                if (reader.read(path.toStdString(), *data->cloud) == -1) {
                    throw std::runtime_error("Invalid PLY file structure");
                }
            }

            // ͳһ����
            postProcessCloud(data->cloud);

            emit cloudReady(data);
        }
        catch (const std::exception& e) {
            emit errorOccurred(tr("�ļ�����ʧ��: %1\nԭ��: %2").arg(path).arg(e.what()));
        }
    }

    /// ���ƺ�������ϵͳһ�ȣ�
    void postProcessCloud(typename pcl::PointCloud<PointT>::Ptr cloud) {
        // �Զ�ת����Z-up����ϵ�����PLY�ļ�ʹ��Y-up��
        if constexpr (std::is_same_v<PointT, pcl::PointXYZ>) {
            for (auto& point : *cloud) {
                float y = point.y;
                point.y = point.z;
                point.z = y;
            }
        }
    }

    /// ��ʼ�����������ļ�
    void processExistingFiles(const QString& dir) {
        QDir directory(dir);
        QStringList filters = { "*.pcd", "*.ply" };
        auto files = directory.entryInfoList(filters, QDir::Files, QDir::Time);

        for (const auto& fileInfo : files) {
            m_watcher.addPath(fileInfo.absoluteFilePath());
            processCloudFile(fileInfo.absoluteFilePath(),
                fileInfo.suffix().toLower());
        }
    }

    QFileSystemWatcher m_watcher;
    QMap<QString, qint64> m_fileTimestamps; // �ļ������ʱ���¼
    int m_minFileInterval; // �ļ�������С�������������
};

#endif // CLOUD_WORKER_H