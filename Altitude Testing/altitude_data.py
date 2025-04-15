
"""
Altitude Data Generation
4/14/2025
"""



import random



def generate_altitude_data(filename, data_points=1000, max_altitude=30000, apogee_index=500):
    with open(filename, 'w') as file:
        altitude = 0  # Start at ground level
        for i in range(data_points):
            if i <= apogee_index:
                # Climbing phase with fluctuations
                altitude += random.uniform(0, 10) - random.uniform(0, 5)  # Small upward trend with noise
            else:
                # Descending phase with fluctuations
                altitude -= random.uniform(0, 10) - random.uniform(0, 5)  # Small downward trend with noise
            
            # Ensure altitude stays within bounds
            altitude = max(0, min(altitude, max_altitude))
            
            file.write(f"{altitude:.2f}\n")



generate_altitude_data('altitude_data.txt')
