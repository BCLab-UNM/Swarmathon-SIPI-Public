#include "sipi_controller/sipi_controller.h"

///////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
  std::string publishedName;
  char host[256];
  gethostname(host, sizeof (host));
  string hostname(host);

  if (argc >= 2) {
    publishedName = argv[1];
    cout << publishedName << ":  sipi_controller_node running" << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }
  cout << "controller HERE\n";
  ros::init(argc, argv, (publishedName + "_BEHAVIOUR") );
  cout << "controller HERE1\n";
  sipi_controller controller(publishedName, argc, argv);
  cout << "controller HERE2\n";
  ros::spin();

  return EXIT_SUCCESS;
}
