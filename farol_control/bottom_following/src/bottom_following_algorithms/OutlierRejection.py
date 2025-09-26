import numpy as np
import rospy

class OutlierRejection:
    """
    Function to perform outlier rejection on a signal vector y of dimension n.
    This is implemented as a class to store the buffer.
    It works by applying a MAD criterion to the data on a rolling window.
    For reference, see Morgado et al., 2014.
    """
    
    def __init__(self, W, threshold, n):
        self.W = W
        self.buffer = np.full((n, W), np.nan)  # Initialize buffer for all channels
        self.idx = 0
        self.count = 0
        self.threshold = threshold
        self.n = n
    
    def compute(self, y):
        y_filtered = np.copy(y)

        # Update buffer
        self.buffer[:, self.idx] = y
        self.idx = (self.idx + 1) % self.W
        self.count = min(self.count + 1, self.W)
        
        # Check if we have enough samples
        if self.count < self.W:
            return y_filtered
        
        # Compute median and MAD
        medians = np.nanmedian(self.buffer, axis=1)
        mad_values = np.nanmedian(np.abs(self.buffer - medians[:, None]), axis=1) / 0.6745
        
        # Identify outliers
        robust_z_scores = np.abs(y - medians) / mad_values
        is_outliers = robust_z_scores > self.threshold
        
        if np.any(is_outliers):
            if self.n == 1:
                rospy.loginfo(f"[Altimeter] Outlier rejected")
            else:
                rospy.loginfo(f"[DVL] Outlier rejected in beams {[i for i, val in enumerate(is_outliers) if val]}")
                
        # Replace outliers with median
        y_filtered[is_outliers] = medians[is_outliers]
        
        return y_filtered
