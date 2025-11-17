#=
Press shift+enter to run lines/blocks of code!
=#

#load the environment and install any packages as required
begin
    using Pkg
    Pkg.activate(".")
    Pkg.instantiate()
end

#define some utility functions and load packages
include("utilities.jl")

begin
  #this makes an Asap structural model of the same cantilever truss
  model = generate_cantilever([0, 0])

  #this defines the x,y positions of node 5 as AsapOptim variables
  variables = [SpatialVariable(model.nodes[5], 0., -1., 1., :X), SpatialVariable(model.nodes[5], 0., -1., 1., :Y)]

  #this defines optimization parameters used in the objective function
  params = TrussOptParams(model, variables)

  #define objective function
  function FL(x, params)

    #analyze the truss with the given variables [x]
    results = solve_truss(x, params)

    #structure properties
    element_lengths = results.L #length of each element
    element_forces = AsapOptim.axial_force(results, params) #axial force of each element
    u = results.U
    P = params.P

    return dot(abs.(element_forces), element_lengths)

  end
end


#the range of design values are already given to you 
dx_range
dy_range

#sampling
loss_matrix = [FL([x, y], params) for x in dx_range, y in dy_range]


#=
Your work starts below
=#

#Here is an example

#a random index
index_of_interest = CartesianIndex(12, 40)

#the function value
loss_of_interest = loss_matrix[index_of_interest]

#extract individual row and column indices
i_row = index_of_interest[1]
j_col = index_of_interest[2]

#get the dx and dy values that correspond to these indices
x_of_interest = dx_range[i_row]
y_of_interest = dy_range[j_col]

#draw the geometry corresponding to dx, dy
fig = draw_cantilever(x_of_interest, y_of_interest)