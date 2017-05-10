uint8 EXACT=0     # Is here for easier msg creation, but is actually a specialized case:
                  # - continuous: is a Gaussian with zero covariance
                  # - discrete: is a Discrete pdf with one entry with P=1                
uint8 GAUSSIAN=1
uint8 UNIFORM=2
uint8 PARTICLES=3
uint8 MIXTURE=4
uint8 DISCRETE=5
uint8 HYBRID=6

uint8 type
uint32 dimensions

# For continuous pdf's
float64[] data

# For discrete pdf's (pmf)
int32 domain_size
string[] values
float64[] probabilities

# For exact values, either a string or a vector
string exact_value_str
float64[] exact_value_vec
