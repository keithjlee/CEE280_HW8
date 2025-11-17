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
end

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



# Optimization algorithms

begin
  #Gradient free methods
  nelder_mead = Optim.NelderMead()

  #evolutionary algorithms
  particle_swarm = Optim.ParticleSwarm()
  simulated_annealing = Optim.SimulatedAnnealing()

  #gradient-based algorithms
  gradient_descent = Optim.GradientDescent()
  conjugate_gradient_descent = Optim.ConjugateGradient()
  LBFGS = Optim.LBFGS()
end


#=
YOUR WORK STARTS BELOW
=#

# choose initial guess
x0 = [.5, -.75]

# choose algorithm
alg = gradient_descent

# visualize initial guess
fig = draw_cantilever(x0)

# optimize (first run will take some extra time)
res = optimize(FL, params, x0, alg);

# visualize design space
begin

    #draw
    fig = Figure()

    landscape = [FL([x, y], params) for x in dx_range, y in dy_range]

    ax = Axis(
      fig[1,1],
      xlabel = "dx [m]",
      ylabel = "dy [m]",
      limits = (-1, 1, -1, 1),
      title = "∑|F|L = $(round(res.f_opt, digits = 2))kNm; time = $(round(res.time, digits = 3))s"
    )

    hm = heatmap!(
      dx_range, dy_range, landscape,
      colormap = :plasma,
      interpolate = true,
      rasterize = 2
    )

    contour!(
      dx_range, dy_range, landscape,
      levels = 25,
      color = (:white, 0.5)
    )

    scatterlines!(Point2.(res.x_history), color = :white)

    #draw start and end points
    scatter!(Point2(first(res.x_history)), color = :black, strokecolor = :black, markersize = 10)
    scatter!(Point2(res.x_opt), color = :white, strokecolor = :black, markersize = 10, strokewidth = 2)

    Colorbar(
    fig[1,2],
    hm,
    label = "Performance [kNm]"
    )

    fig
end

#visualize solution
fig = draw_cantilever(res.x_opt)
