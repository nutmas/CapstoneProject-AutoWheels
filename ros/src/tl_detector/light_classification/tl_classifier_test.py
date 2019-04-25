import os
import cv2
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
from tl_classifier import TLClassifier
from sets import Set

# The path to the hand cropped training image files.
TL_TRAINING_IMAGES_BASE_PATH = "./traffic_light_training_images" 

def read_training_data(base_path):
	"""Reads in all training images and labels from files at base path.

	Args:
		base_path: The file directory containing training images.

	Returns:
		An Iterable[cv::Mat, image_id: int, image_label] containing training images,
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
		training_images.append([cv2.imread(file), image_id, color])
	return training_images

def parse_label_from_image_name(image_name):
	"""Parses the label from the file name of a training image.

	Args:
		image_name: A file name string of the following format.
			<training_image_id>_<training_image_label> where
			training_image_id is a unique integer amongst training
			images and training_image_label is a valid traffic light color
			[red, green, yellow, unknown].

	Returns:
		[training_image_id: int, color: int] where color
		is specified in styx_msgs/TrafficLight.
	"""
	name_components = image_name.split("_")
	assert len(name_components) == 2
	id_str, color_str = name_components
	image_id = int(id_str)
	color = TrafficLight.UNKNOWN
	switch (color_str):
		case "red":
			color = TrafficLight.RED
			break
		case "yellow":
			color = TrafficLight.YELLOW
			break
		case "green":
			color = TrafficLight.GREEN
			break
		default:
			break
	return image_id, color

def get_color_name(color_int):
	"""Gets a human readable name for a traffic sign color.

	Args:
		color_int: ID of traffic light color (specified in styx_msgs/TrafficLight).

	Returns:
		A string corresponding to the color name.
	"""
	switch (color_int):
		case 0:
			return "RED"
		case 1:
			return "YELLOW"
		case 2:
			return "GREEN"
		default:
			return "UNKNOWN"

def main():
	training_data = read_training_data(TL_TRAINING_IMAGES_BASE_PATH)
	clf = TLClassifier()
	labels = []
	predictions = []

	for image, image_id, color_label in training_data:
		color_prediction = clf.get_classification(image)
		labels.append(color_label)
		predictions.append(color_prediction)
		print ("Image: ", image_id, 
			   " Label: ", get_color_name(color_label), 
			   " Prediction: " get_color_name(color_prediction))


	print "Accuracy: ", accuracy_score(labels, predictions)
	print "Confusion Matrix: ", confusion_matrix(labels, predictions)


if __name__ == '__main__':
	main()