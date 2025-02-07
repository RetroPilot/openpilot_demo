#include "selfdrive/ui/qt/api.h"

#include <QDateTime>
#include <QDebug>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRandomGenerator>
#include <QString>
#include <QTimer>
#include <QWidget>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/rsa.h>

#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

const std::string private_key_path =
    Hardware::PC() ? util::getenv_default("HOME", "/.comma/persist/comma/id_rsa", "/persist/comma/id_rsa")
                   : "/persist/comma/id_rsa";

QByteArray CommaApi::rsa_sign(const QByteArray &data) {
  auto file = QFile(private_key_path.c_str());
  if (!file.open(QIODevice::ReadOnly)) {
    qDebug() << "No RSA private key found, please run manager.py or registration.py";
    return QByteArray();
  }
  auto key = file.readAll();
  file.close();
  file.deleteLater();
  
  // Read the private key
  BIO* mem = BIO_new_mem_buf(key.data(), key.size());
  assert(mem);
  EVP_PKEY* pkey = PEM_read_bio_PrivateKey(mem, NULL, NULL, NULL);
  assert(pkey);
  
  // Prepare signature
  QByteArray sig;
  sig.resize(EVP_PKEY_size(pkey));  // Use EVP_PKEY_size for key size
  
  // Sign the data
  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  EVP_SignInit(ctx, EVP_sha256());
  EVP_SignUpdate(ctx, data.data(), data.size());
  unsigned int sig_len;
  int ret = EVP_SignFinal(ctx, (unsigned char*)sig.data(), &sig_len, pkey);
  assert(ret == 1);
  assert(sig_len == sig.size());
  
  EVP_MD_CTX_free(ctx);
  EVP_PKEY_free(pkey);  // Free the EVP_PKEY object (no need to call RSA_free anymore)
  BIO_free(mem);
  
  return sig;
}

QString CommaApi::create_jwt(const QVector<QPair<QString, QJsonValue>> &payloads, int expiry) {
  QString dongle_id = QString::fromStdString(Params().get("DongleId"));

  QJsonObject header;
  header.insert("alg", "RS256");

  QJsonObject payload;
  payload.insert("identity", dongle_id);

  auto t = QDateTime::currentSecsSinceEpoch();
  payload.insert("nbf", t);
  payload.insert("iat", t);
  payload.insert("exp", t + expiry);
  for (auto &load : payloads) {
    payload.insert(load.first, load.second);
  }

  auto b64_opts = QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals;
  QString jwt = QJsonDocument(header).toJson(QJsonDocument::Compact).toBase64(b64_opts) + '.' +
                QJsonDocument(payload).toJson(QJsonDocument::Compact).toBase64(b64_opts);

  auto hash = QCryptographicHash::hash(jwt.toUtf8(), QCryptographicHash::Sha256);
  auto sig = rsa_sign(hash);
  jwt += '.' + sig.toBase64(b64_opts);
  return jwt;
}

HttpRequest::HttpRequest(QObject *parent, const QString &requestURL, const QString &cache_key, bool create_jwt_) : cache_key(cache_key), create_jwt(create_jwt_), QObject(parent) {
  networkAccessManager = new QNetworkAccessManager(this);
  reply = NULL;

  networkTimer = new QTimer(this);
  networkTimer->setSingleShot(true);
  networkTimer->setInterval(20000);
  connect(networkTimer, &QTimer::timeout, this, &HttpRequest::requestTimeout);

  sendRequest(requestURL);

  if (!cache_key.isEmpty()) {
    if (std::string cached_resp = Params().get(cache_key.toStdString()); !cached_resp.empty()) {
      QTimer::singleShot(0, [=]() { emit receivedResponse(QString::fromStdString(cached_resp)); });
    }
  }
}

void HttpRequest::sendRequest(const QString &requestURL){
  QString token;
  if(create_jwt) {
    token = CommaApi::create_jwt();
  } else {
    QString token_json = QString::fromStdString(util::read_file(util::getenv_default("HOME", "/.comma/auth.json", "/.comma/auth.json")));
    QJsonDocument json_d = QJsonDocument::fromJson(token_json.toUtf8());
    token = json_d["access_token"].toString();
  }

  QNetworkRequest request;
  request.setUrl(QUrl(requestURL));
  request.setRawHeader(QByteArray("Authorization"), ("JWT " + token).toUtf8());

  reply = networkAccessManager->get(request);

  networkTimer->start();
  connect(reply, &QNetworkReply::finished, this, &HttpRequest::requestFinished);
}

void HttpRequest::requestTimeout(){
  reply->abort();
}

// This function should always emit something
void HttpRequest::requestFinished(){
  if (reply->error() != QNetworkReply::OperationCanceledError) {
    networkTimer->stop();
    QString response = reply->readAll();

    if (reply->error() == QNetworkReply::NoError) {
      // save to cache
      if (!cache_key.isEmpty()) {
        Params().put(cache_key.toStdString(), response.toStdString());
      }
      emit receivedResponse(response);
    } else {
      if (!cache_key.isEmpty()) {
        Params().remove(cache_key.toStdString());
      }
      qDebug() << reply->errorString();
      emit failedResponse(reply->errorString());
    }
  } else {
    emit timeoutResponse("timeout");
  }
  reply->deleteLater();
  reply = NULL;
}
