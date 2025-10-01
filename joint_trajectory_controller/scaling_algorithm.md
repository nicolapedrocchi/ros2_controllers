Hi, sorry for the late reply
OK, I've always worked in quasi-static applications, anyway, I agree with you. A simple (brutal) scaling algorithm could be the following: 

Let:

- `s₂(t) = s₁(τ)` (1)  
- `v₂(t) = v₁(τ) τ̇` (2)  
- `a₂(t) = a₁(τ) τ̇² + v₁(τ) τ̈` (3)

Where:

- `τ̇` is the target scaling factor received as input for the system.

The system must satisfy the following constraints:

- `0 ≤ τ̇ ≤ 1.0` (5)  
- `−v_max ≤ v₁(τ) τ̇ ≤ v_max` (6)  
- `−a_max ≤ a₁(τ) τ̇² + v₁(τ) τ̈ ≤ a_max` (7)

Therefore

`i ← i + 1` // The superscript tracks the control loop iteration  
`τ̇ᵢ ← SATURATION(EXP_FILTER(target_scaling))`  
`OK ← False`  
**while** `not OK` **do**  
    `τᵢ ← τᵢ₋₁ + τ̇ᵢ (tᵢ − tᵢ₋₁)`  
    `k, k₋₁ ← get_trajectory_interval(τᵢ)`  
    **if** `v_k(τᵢ) τ̇ᵢ ≤ v_max` **then**  
      `τ̈ᵢ ← (τ̇ᵢ − τ̇ᵢ₋₁) / (tᵢ − tᵢ₋₁)`  
      **if** `a_k(τ) τ̇² + v_k(τ) τ̈ ≤ a_max` **then**  
        `OK ← True`  
      **else**  
        `τ̇ᵢ ← 0.9 τ̇ᵢ`  // THIS IS BRUTAL, but in about 20 iterations it should converge in any condition
      **end if**  
    **else**  
     `τ̇ᵢ ← v_max / v_k(τᵢ)`  
    **end if**  
**end while**


If it is OK with you, I can implement it and test it (I have already done some preliminary evaluations in my fork) - the PR I guess, will consist of: 
- functions to read the limits (taking as examples other controllers)
- manage the filter param
- adding a header/src with the main code of the algorithm
- implements all the tests (maybe adding a test in the unit)

However, I have some doubts about the code regarding the state interface you used for the scaling and its relation to the publisher. 
Should this controller be chainable? In such a case, the PR will be quite invasive since I guess that I have to split the controller update function into two functions. Am I right?
