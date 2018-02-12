#include "sipi_controller/sipi_controller.h"

///////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
  std::string publishedName;
  char host[256];
  gethostname(host, sizeof (host));
  string hostname(host);

  if (argc >= 2) {
    publishedName = argv[1];
    cout << publishedName << ":  Mobility" << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }
  ros::init(argc, argv, (publishedName + "_MOBILITY") );
  sipi_controller controller(publishedName, argc, argv);
  ros::spin();

  return EXIT_SUCCESS;
}
