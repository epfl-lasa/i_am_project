import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import time
from sklearn.linear_model import LinearRegression


def plot_distance_vs_flux(processed_csv, with_linear_regression=True):
    
    df = pd.read_csv(processed_csv)

    # Plot Flux
    fig, ax = plt.subplots(1, 1, figsize=(10, 4), sharex=True)
    ax.scatter(df['HittingFlux'], df['DistanceTraveled'], color='blue', label='Data points')

    ## Add linear regression
    if with_linear_regression: 
        model = LinearRegression()
        model.fit(df['HittingFlux'].values.reshape(-1,1), df['DistanceTraveled'].values)

        flux_test = np.linspace(0.4,1.2,100).reshape(-1,1)
        distance_pred = model.predict(flux_test)
        ax.plot(flux_test,distance_pred,color='red', label='Predictions')

    # Print some infos
    low_to_med_threshold = 0.65
    med_to_high_threshold = 0.85
    print(f"Dataset info : \n"
          f" Low Flux points (below {low_to_med_threshold}): {len(df[df['HittingFlux'] < low_to_med_threshold].index)} \n"
          f" Medium Flux points : {len(df[df['HittingFlux'] > low_to_med_threshold].index) - len(df[df['HittingFlux'] > med_to_high_threshold].index)} \n"
          f" High Flux points (above {med_to_high_threshold}) : {len(df[df['HittingFlux'] > med_to_high_threshold].index)} \n"
          f" Total points : {len(df.index)}")

    ax.set_xlabel('Hitting flux [m/s]')
    ax.set_ylabel('Distance Traveled [m]')
    ax.grid(True)
    plt.legend()
    fig.suptitle(f"Distance over Flux")
    fig.tight_layout(rect=(0.01,0.01,0.99,0.99))

    plt.show()



if __name__== "__main__" :
   
    processed_data_folder = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+ "/data/airhockey_processed/"
    csv_fn = processed_data_folder+ "all_data_march.csv"
    plot_distance_vs_flux(csv_fn)
