#=
Press shift+enter to run lines/blocks of code!
=#

using LinearAlgebra, Asap, AsapOptim, Zygote, CairoMakie
using Optimization, OptimizationOptimJL

#define some utility functions
include("Problem2_utilities.jl")

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

# visualize initial guess
fig = draw_cantilever(x0)

# choose algorithm
alg = LBFGS

# optimize
res = optimize(FL, params, x0, alg);

# visualize design space
begin

    #draw
    fig = Figure()

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

#best solution
fl_opt = res.f_opt

i_okay = findall(landscape .<= 1.50fl_opt)
i_sample = rand(i_okay, 10)

axis_index = reshape(1:10, 2, 5)

nrows, ncols = size(axis_index)

begin
fig = Figure(size = (600, 200))
  for i = 1:nrows
    for j = 1:ncols
      index = i_sample[axis_index[i, j]]
      dx = dx_range[index[1]]
      dy = dy_range[index[2]]

      model = generate_cantilever([dx, dy])
      pts, els = model_to_geometry(model)

      ax = Axis(
        fig[i, j],
        aspect = DataAspect(),
        limits = (-.25, 4.25, -3.25, .25)
      )

      hidedecorations!(ax)
      hidespines!(ax)

      linesegments!(els, color = :black)
      scatter!(pts, color = :white, strokecolor = :black, strokewidth = 1, markersize = 5)
    end
  end

  # rowgap!(fig.layout, 1, )
  # resize_to_layout!(fig)
  fig
end
