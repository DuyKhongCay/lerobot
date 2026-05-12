from lerobot.datasets import LeRobotDataset

# Load the push-t dataset from Hugging Face hub
dataset = LeRobotDataset("lerobot/pusht")

# Extract the schema/blueprint of the dataset
dataset_schema = dataset.meta.features

# Print out the defined features
for feature_name, feature_details in dataset_schema.items():
    # Print the name of the column and its specific configuration
    print(f"Column: {feature_name}")
    print(f"Details: {feature_details}")
    print("-" * 20)