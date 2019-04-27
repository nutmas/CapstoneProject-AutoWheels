import os
import cv2
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
from tl_classifier import TLClassifier
from sets import Set

# The path to the hand cropped testing image files.
TL_TESTING_IMAGES_RELATIVE_PATH = os.path.join(os.getcwd(), "traffic_light_testing_images" )

def read_training_data(base_path):
	"""Reads in all testing images and labels from files at base path.

	Args:
		base_path: The file directory containing testing images.

	Returns:
		An Iterable[cv::Mat, image_id: int, image_label] containing testing images,
		their unique int ids and their labels. The label is a valid traffic light color
			[red, green, yellow, unknown].

	"""
	training_images = []
	image_ids = Set()
	for file in os.listdir(base_path):
		image_id, color = parse_label_from_image_name(file)
		# Ensures that the same image is never reused.
		assert image_id not in image_ids
		image_ids.add(image_id)
		training_images.append([cv2.imread(os.path.join(TL_TESTING_IMAGES_RELATIVE_PATH, file)), image_id, color])
	return training_images

def parse_label_from_image_name(image_name):
	"""Parses the label from the file name of a testing image.

	Args:
		image_name: A file name string of the following format.
			<testing_image_id>_<training_image_label> where
			training_image_id is a unique integer amongst testing
			images and training_image_label is a valid traffic light color
			[red, green, yellow, unknown].

	Returns:
		[training_image_id: int, color: int] where color
		is specified in styx_msgs/TrafficLight.
	"""
	name_components = image_name.split(".")[0].split("_")
	assert len(name_components) == 2
	id_str, color_str = name_components
	image_id = int(id_str)
	color_str_map = {
		"red": 0,
		"yellow": 1,
		"green": 2,
		"unknown": 4,
	}
	return image_id, color_str_map.get(color_str, 4)

def get_color_name(color_int):
	"""Gets a human readable name for a traffic sign color.

	Args:
		color_int: ID of traffic light color (specified in styx_msgs/TrafficLight).

	Returns:
		A string corresponding to the color name.
	"""
	color_int_map = {
		0: "RED",
		1: "YELLOW",
		2: "GREEN",
		4: "UNKNOWN",
	}
	return color_int_map.get(color_int, "UNKNOWN")

def main():
	testing_data = read_training_data(TL_TESTING_IMAGES_RELATIVE_PATH)
	clf = TLClassifier()
	labels = []
	predictions = []
	print "Testing"
	for image, image_id, color_label in testing_data:
		color_prediction = clf.get_classification(image)
		labels.append(color_label)
		predictions.append(color_prediction)

		print ("Image: ", image_id, 
			   " Label: ", get_color_name(color_label), 
			   " Prediction: ", get_color_name(color_prediction))


	print ("Accuracy: ", accuracy_score(labels, predictions))
	print ("Total: ", len(labels), " Correct: ", accuracy_score(labels, predictions, normalize=False))
	print ("Confusion Matrix: ", confusion_matrix(labels, predictions))


if __name__ == '__main__':
	main()