import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ==========================
# Cargar CSV
# ==========================
df = pd.read_csv("ppr_left.csv")

# convertir tiempo
df["timestamp"] = pd.to_datetime(df["timestamp"])

# ==========================
# limpiar datos
# ==========================
df_clean = df[(df["pulses"] > 1500) & (df["pulses"] < 2500)]

# ==========================
# estadisticas
# ==========================
mean_val = df_clean["pulses"].mean()
median_val = df_clean["pulses"].median()
std_val = df_clean["pulses"].std()

print("==== Estadísticas ====")
print(f"Media: {mean_val:.2f}")
print(f"Mediana: {median_val:.2f}")
print(f"Desviación estándar: {std_val:.2f}")

# ==========================
# grafica temporal
# ==========================
plt.figure(figsize=(10,5))

plt.plot(
    df["timestamp"].to_numpy(),
    df["pulses"].to_numpy(),
    "o-",
    label="mediciones"
)

plt.axhline(mean_val, linestyle="--", label="media")
plt.axhline(median_val, linestyle=":", label="mediana")

plt.title("Pulsos por revolución")
plt.xlabel("Tiempo")
plt.ylabel("Pulsos")
plt.legend()
plt.grid()

plt.show()

# ==========================
# histograma
# ==========================
plt.figure(figsize=(8,5))

plt.hist(df_clean["pulses"].to_numpy(), bins=20)

plt.axvline(mean_val, linestyle="--", label="media")
plt.axvline(median_val, linestyle=":", label="mediana")

plt.title("Distribución de pulsos")
plt.xlabel("Pulsos")
plt.ylabel("Frecuencia")

plt.legend()
plt.grid()

plt.show()

# ==========================
# boxplot
# ==========================
plt.figure(figsize=(6,4))

plt.boxplot(df["pulses"].to_numpy(), vert=False)

plt.title("Boxplot pulsos encoder")
plt.xlabel("Pulsos")

plt.show()