# PID debug message

# Header information
Header header

# Controller in use
string controller
# Reference
float64 ref
# Reference derivative
float64 ref_d
# Reference derivative filter
float64 ref_d_filtered
# Actual state value
float64 state
# Error between ref and actual state value
float64 error
# Error saturated
float64 error_saturated

# Feed forward term 
float64 ffTerm
# Feed forward derivative term
float64 ffDTerm
# Feed forward drag term
float64 ffDragTerm

# Proportional term
float64 pTerm
# Integral Term
float64 iTerm
# Derivative Term
float64 dTerm
# Actual value
float64 output
