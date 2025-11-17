# CLICK THIS LINE AND PRESS SHIFT+ENTER TO RUN A LINE OF CODE AND GO TO THE NEXT LINE


# Load the current environment and install all necessary packages (first run will take a while)
begin
    using Pkg
    Pkg.activate(".")
    Pkg.instantiate()
end

#define some utility functions and load packages
include("utilities.jl")

#=
PROBLEM 2
=#

"""
    R_element(cx, cy)

Given:
cx: the x component of the normalized element vector, or cos(θₑ), where θₑ is the angle of the element with respect to the global X axis
cy: the y component of the normalized element vector

returns the [2 × 4] transformation matrix of a given element.
"""
function R_element(cx, cy)
    return [cx cy 0 0;
            0 0 cx cy]
end

"""
    k_element(E, A, L)

Given:
E: Modulus of Elasticity
A: Cross section area
L: Element length

returns the [2×2] truss stiffness matrix in local coordinates.
"""
function k_element(E, A, L)

    #YOUR CODE HERE (replace nothing with your stiffness matrix)
    k = nothing

    return k
end

"""
    FL(dx, dy)

Given:
dx: Change of node 5 position in x direction
dy: Change of node 5 position in y direction

Performs structural analysis and outputs the performance score:

∑|Fᵢ|Lᵢ

"""
function FL(dx, dy)

  # constants
  E = 200e6 #kN/m2
  A = 1e-3 #m2
  P = 50 #kN

  #=
  NODES
  =#

  # node positions defined as vectors [x, y]
  p1 = [0, 0]
  p2 = [2, 0]
  p3 = [4, 0]
  p4 = [0, -2]

  # remember that the position of node 5 is a variable, so we add dx and dy to the respective entries
  p5 = [2+dx, -2+dy]

  #collect node positions
  positions = [p1, p2, p3, p4, p5]
  n_nodes = length(positions)

  #=
  DEGREES OF FREEDOM
  =#

  n_dofs = n_nodes * 2

  #get degrees of freedom
  dofs_per_node = [[1,2], [3,4], [5,6], [7,8], [9,10]] #e.g. node 1 has dofs dofs_per_node[1] = [1,2]

  #=
  ELEMENTS
  =#

  #element vectors
  element_connectivities = [[1,2], [2,3], [1,5], [5,2], [5,3], [4,5]]
  element_dofs = [[1,2,3,4], [3,4,5,6], [1,2,9,10], [9,10,3,4], [9,10,5,6], [7,8,9,10]]
  n_elements = length(element_connectivities)

  #get element properties
  element_vectors = []
  normalized_element_vectors = []
  element_lengths = []

  #populate for each element
  for connectivity in element_connectivities #for each item in element_connectivities

      #get the start and end node indices for the element
      istart = connectivity[1]
      iend = connectivity[2]

      #compute the vector v = p_end - p_start
      element_vector = positions[iend] - positions[istart]
      normalized_element_vector = normalize(element_vector)

      #compute the length of the element (norm of vector)
      element_length = norm(element_vector)

      #add to lists
      push!(element_vectors, element_vector)
      push!(normalized_element_vectors, normalized_element_vector)
      push!(element_lengths, element_length)
  end

  #get stiffness matrices
  local_stiffness_matrices = [k_element(E, A, l) for l in element_lengths]

  #get transformation matrices
  transformation_matrices = [R_element(v[1], v[2]) for v in normalized_element_vectors]

  #=
  GLOBAL STIFFNESS
  =#

  #get global stiffness matrices
  global_coordinate_stiffness_matrices = [r' * k * r for (r, k) in zip(transformation_matrices, local_stiffness_matrices)]

  #initiate global stiffness matrix
  K = zeros(n_dofs, n_dofs)

  #populate with element stiffness matrices
  for i = 1:n_elements
      dof = element_dofs[i]
      k = global_coordinate_stiffness_matrices[i]

      K[dof, dof] .+= k
  end

  #=
  LOAD
  =#

  # initialize a load vector 
  load_vector = zeros(n_dofs)

  # add load values to 
  load_vector[4] = -P
  load_vector[6] = -P


  #=
  SOLVING
  =#

  #DOFs that are active in the solution
  free_indices = [3, 4, 5, 6, 9, 10]

  #free DOF stiffness matrix
  K_free = K[free_indices, free_indices]

  #free load vector
  P_free = load_vector[free_indices]

  #solve for free displacements
  u_free = K_free \ P_free

  #=
  POST PROCESSING
  =#

  # full displacement vector
  u = zeros(n_dofs)

  # populate free displacements
  u[free_indices] .= u_free

  # get displacements per node
  u_per_node = [u[dof] for dof in dofs_per_node]

  # get displacements for each element end points
  u_per_element = [u[dof] for dof in element_dofs]

  # get axial forces in each element
  element_forces = []
  for i = 1:n_elements
    re = transformation_matrices[i]
    ke = global_coordinate_stiffness_matrices[i]
    ue = u_per_element[i]

    F = (re * ke * ue)[2]
    push!(element_forces, F)
  end

  #=
  PERFORMANCE
  =#

  F = element_forces #vector of element internal forces
  L = element_lengths #vector of element lengths
  P = load_vector #vector of external loads
  u = u #vector of nodal displacements


  #YOUR CODE HERE (replace loss with the actual loss value)
  loss = 0

  return loss
end

#test your code
isapprox(FL(0.25,0.25), 1178.5714285714275)

#=
SAMPLING
=#

n_samples = 50
dx_range = range(-1, 1, n_samples)
dy_range = range(-1, 1, n_samples)

p_v_x = [FL(x, 0) for x in dx_range]
x_min = dx_range[argmin(p_v_x)]

#visualize
begin
    fig = Figure()
    ax = Axis(
    fig[1,1],
    xlabel = "dx [m]",
    ylabel = "∑|F|L [kNm]"
    )

    ylims!(0, 1800)

    lines!(
    dx_range, p_v_x,
    color = :black
    )

    vlines!(
        [x_min],
        color = :gray,
        linestyle = :dash
    )

    fig
end

#uncomment the line below to save your figures as a .pdf. Or replace the ending with ".jpg" or ".png"
# save("figure1.pdf", fig)

# Test dy
p_v_y = [FL(0, y) for y in dy_range]
y_min = dy_range[argmin(p_v_y)]

begin
    fig = Figure()
    ax = Axis(
    fig[1,1],
    xlabel = "dy [m]",
    ylabel = "∑|F|L [kNm]"
    )

    ylims!(0, 1800)

    lines!(
    dy_range, p_v_y,
    color = :black
    )

    vlines!(
        [y_min],
        color = :gray,
        linestyle = :dash
    )

    fig
end

# test full design space
p_v_xy = zeros(n_samples, n_samples)

for i = 1:n_samples
  for j = 1:n_samples
    _x = dx_range[i]
    _y = dy_range[j]

    perf = FL(_x, _y)
    p_v_xy[i,j] = perf
  end
end

i_opt = argmin(p_v_xy)
x_opt = dx_range[i_opt[1]]
y_opt = dy_range[i_opt[2]]


begin
    fig = Figure()
    ax = Axis(
    fig[1,1],
    xlabel = "dx [m]",
    ylabel = "dy [m]"
    )

    hm = heatmap!(
        dx_range, dy_range, p_v_xy,
        colormap = :plasma,
        interpolate = true,
        rasterize = 2
    )

    scatter!(
        [x_opt], [y_opt],
        color = :white
    )

    contour!(
    dx_range, dy_range, p_v_xy,
    levels = 25,
    color = :white
    )

    Colorbar(
    fig[1,2],
    hm,
    label = "∑|F|L [kNm]"
    )

    fig
end

fig = draw_cantilever([x_opt, y_opt])