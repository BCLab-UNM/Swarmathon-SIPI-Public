#include <ros/ros.h>
#include "VisualLocalization.h"
using namespace std;
int main(int argc, char **argv) {
	char host[128];
	string publishedName;

	gethostname(host, sizeof (host));
	string hostname(host);
	if (argc >= 2) {
		publishedName = argv[1];
	} else {
		publishedName = hostname;
  }
	cout << "Visual localization running: Name= " << publishedName << endl;

	// NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
	ros::init(argc, argv, (publishedName + "_VISUAL"), ros::init_options::NoSigintHandler);
	ros::NodeHandle mNH;
	CvisualLocalization v(publishedName, mNH);

	ros::spin();

	return EXIT_SUCCESS;
}
