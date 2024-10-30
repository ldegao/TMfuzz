import tensorflow as tf

print("TensorFlow version:", tf.__version__)
print("Num GPUs Available: ", len(tf.config.experimental.list_physical_devices('GPU')))
hello = tf.constant('Hello, TensorFlow!')

tf.print(hello)
