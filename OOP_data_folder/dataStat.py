import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load the CSV file into a DataFrame
file_path = 'OOP_data_folder/OOP_data1.csv'  # Replace with your actual file path
df = pd.read_csv(file_path)

# Calculate the percentage of successful results
success_count = df['result'].sum()
total_count = len(df)
success_percentage = (success_count / total_count) * 100
print(f"Percentage of successful results: {success_percentage:.2f}%")

# Visualize the percentage of successful results
plt.figure(figsize=(6, 6))
labels = ['Success (1)', 'Failure (0)']
sizes = [success_count, total_count - success_count]
colors = ['#76c893', '#f94144']
explode = (0.1, 0)  # Highlight the 'Success' slice
plt.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=140, colors=colors, explode=explode)
plt.title('Percentage of Successful Results')
plt.savefig('success_percentage_pie_chart.png')  # Save the pie chart
plt.show()

# Create subplots for the distributions of each parameter
parameters = ['x', 'y', 'z', 'x_ori', 'y_ori', 'z_ori']
num_parameters = len(parameters)

# Set up the figure and axes
fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(15, 10))
axes = axes.flatten()  # Flatten the 2D array of axes for easy indexing

for i, param in enumerate(parameters):
    sns.histplot(df[param], kde=True, ax=axes[i], bins=20, color='skyblue')
    axes[i].set_title(f"Distribution of {param}")
    axes[i].set_xlabel(param)
    axes[i].set_ylabel("Frequency")

# Remove any empty subplots (if applicable)
for j in range(len(parameters), len(axes)):
    fig.delaxes(axes[j])

# Adjust layout for better spacing and save the figure
plt.tight_layout()
plt.savefig('parameter_distributions.png')  # Save the distributions plot
plt.show()
