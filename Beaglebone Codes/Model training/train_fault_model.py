import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier
from sklearn.multioutput import MultiOutputClassifier
import joblib

# Load dataset
df = pd.read_csv("cleaned_dataset.csv")

# Define input and output
X = df.drop(columns=['Fault', 'Engine_Overheat', 'Overspeed', 'High_RPM', 'Battery_Critical'])
X = X.astype({col: int for col in X.select_dtypes('bool').columns})  # Convert bools to int
y = df[['Engine_Overheat', 'Overspeed', 'High_RPM', 'Battery_Critical']].astype(int)

# Split dataset
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Train a multi-output Decision Tree
model = MultiOutputClassifier(DecisionTreeClassifier(max_depth=6, random_state=42))
model.fit(X_train, y_train)

# Save the model
joblib.dump(model, "fault_type_model.joblib")
print(" Model training complete! Saved as fault_type_model.joblib")
