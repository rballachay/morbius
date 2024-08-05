#include <pistache/endpoint.h>

using namespace Pistache;

struct HelloHandler : public Http::Handler {
  HTTP_PROTOTYPE(HelloHandler)
  void onRequest(const Http::Request&, Http::ResponseWriter writer) override {
    writer.send(Http::Code::Ok, "Hello, World!");
  }
};

int main() {
  Http::listenAndServe<HelloHandler>(Pistache::Address("*:9080"));
}