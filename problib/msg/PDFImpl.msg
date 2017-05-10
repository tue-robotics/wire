uint8 GAUSSIAN=1
uint8 UNIFORM=2
uint8 DISJUNCTION_STRING = 3  # PMF
uint8 DISJUNCTION_FLOAT = 4   # PMF
uint8 DISJUNCTION_PDF = 5     # Mixture
uint8 CONJUNCTION = 6

uint8 type
uint32 num_dimensions
DimensionDescription[] dimensions

# Gaussian
float64[] mean
float64[] covariance

# Uniform
float64 density

# For both mixture and discrete 
float64[] probabilities

# mixture of PFDs (conjunction or disjunction)
uint32[] sub_pdf_ptrs

# discrete
int32 domain_size
float64[] values_float
string[] values_string

