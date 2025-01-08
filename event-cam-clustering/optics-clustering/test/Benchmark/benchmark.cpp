// Copyright Ingo Proff 2016.
// https://github.com/CrikeeIP/OPTICS-Clustering
// Distributed under the MIT Software License (X11 license).
// (See accompanying file LICENSE)


#include "../../include/optics/optics.hpp"
#include "../../include/optics/Stopwatch.hpp"

#include <random>

namespace sw = stopwatch;


//static const std::random_device rd;
static std::mt19937 gen( 1 );


template <typename T, typename = typename std::enable_if<std::is_integral<T>::value >::type >
std::uniform_int_distribution<T> uniform_distribution( T a, T b )
{
	return std::uniform_int_distribution<T>( a, b );
}

template <typename T, typename = typename std::enable_if<std::is_floating_point<T>::value >::type >
std::uniform_real_distribution<T> uniform_distribution( T a, T b )
{
	return std::uniform_real_distribution<T>( a, b );
}


template<std::size_t dimension, typename type, typename distribution>
std::array<type, dimension> get_random_point( distribution& dis ) {
	std::array<type, dimension> result;
	for ( std::size_t d = 0; d < dimension; d++ ) {
		result[d] = dis( gen );
	}
	return result;
}


//Returns the estimated space for each point given a perfect uniform distribution of n_points in dim dimensional volume space_volume
void eps_guess( double space_volume, std::size_t n_points, std::size_t dim ) {
	double d = static_cast<double> (dim);
	const auto get_sphere_volume = [d]( double radius ) -> double {
		double nom = std::pow( geom::pi, d / 2.0 ) * std::pow( radius, d );
		double denom = std::tgamma( d / 2.0 + 1.0 );
		double V = nom / denom;
		return V;

	};
	double nominator = space_volume * 1.0 * std::tgamma( d / 2.0 + 1.0 );
	double denominator = static_cast<double>(n_points) * std::pow( geom::pi, d / 2.0 );
	double r = std::pow( nominator / denominator, 1.0 / d );
	std::cout << "Estimated distance between two points: " << r << std::endl;
	double V = get_sphere_volume( r );
	std::cout << "Estimated volume per point: " << V
		<< " ->V=" << V*n_points << std::endl;
}


template<std::size_t n_points, std::size_t dimension, std::uint64_t space_volume, typename type, std::size_t laps, std::size_t min_pts, std::size_t n_threads>
std::uint64_t test( std::ofstream& output, double epsilon = -1.0 ) {
	static_assert(dimension > 0, "Dimension must be >0");
	static_assert(space_volume > 0, "Volume must be >0");

	std::cout << "\n\nDims = " << dimension << " n_points = " << n_points << " min_pts = " << min_pts  << std::endl;
	//Space parameters
	double edge_length = std::pow( static_cast<double>(space_volume), 1.0 / static_cast<double>(dimension) );
	std::cout << "Space volume: " << space_volume << std::endl;
	std::cout << "Space edge length: " << edge_length
		<< " ->V=" << std::pow( edge_length, static_cast<double>(dimension) ) << std::endl;
	eps_guess( static_cast<double>(space_volume), n_points, dimension );

	//Create a distribution that outputs coordinates between 0 and edge_length
	auto dis = uniform_distribution<type>( 0, static_cast<type>(edge_length) );

	//Create n_points random points
	std::vector<std::array<type, dimension>> points;
	points.reserve( n_points );
	for ( std::size_t n = 0; n <= n_points - 1; n++ ) {
		points.push_back( get_random_point<dimension, type>( dis ) );
	}

	//Let's go
	std::cout << std::endl << "Starting " << laps << " computations of  optics::compute_reachability_dist() ..." << std::endl;
	sw::Stopwatch watch;
	for ( std::size_t lap = 1; lap <= laps; lap++ ) {
		if ( laps < 10 || (lap % (laps / 10) == 0) ) std::cout << lap << "..";
		optics::compute_reachability_dists<n_points, type, dimension, n_threads>( points, min_pts, epsilon );
		watch.lap();
	}

	auto elapsed = watch.elapsed_laps();
	std::cout << std::endl;
	std::cout << "Lap times [ms]: " << sw::show_times( elapsed.second ) << std::endl;
	std::cout << "Total time: " << elapsed.first << "ms" << std::endl;
	std::cout << "Mean time: " << elapsed.first / laps << "ms" << std::endl;

	auto result = elapsed.first / laps;
	output << dimension << ";" << n_points << ";" << space_volume << ";" << min_pts << ";" << result << std::endl;
	return result;
}



constexpr std::array<std::size_t, 27> dimensions = { 1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 16, 20, 25, 30, 32, 50, 64, 65, 100, 128, 129, 256, 257, 512,513, 1024, 1025  };

int test_all() {
	std::ofstream output;
	output.open( "benchmark.csv" );
	output << "dimension;n_points;volume;min_clustersize;time" << std::endl;

	//dim = 1
	{
		//n_pts = 1000
	   test<1000, 1, 1, double, 2, 10, 1>( output );
	   test<1000, 1, 1, double, 2, 20, 1>( output );
	   test<1000, 1, 1, double, 2, 50, 1>( output );
	   test<1000, 1, 1, double, 2, 100, 1>( output );
		//n_pts = 5000
	   test<5000, 1, 1, double, 2, 10, 1>( output );
	   test<5000, 1, 1, double, 2, 20, 1>( output );
	   test<5000, 1, 1, double, 2, 50, 1>( output );
	   test<5000, 1, 1, double, 2, 100, 1>( output );
		//n_pts = 10000
	   test<10000, 1, 1, double, 2, 10, 1>( output );
	   test<10000, 1, 1, double, 2, 20, 1>( output );
	   test<10000, 1, 1, double, 2, 50, 1>( output );
	   test<10000, 1, 1, double, 2, 100, 1>( output );
		//n_pts = 50000
	   test<50000, 1, 1, double, 2, 10, 1>( output );
	   test<50000, 1, 1, double, 2, 20, 1>( output );
	   test<50000, 1, 1, double, 2, 50, 1>( output );
	   test<50000, 1, 1, double, 2, 100, 1>( output );
	   test<50000, 1, 1, double, 2, 500, 1>( output );
		//n_pts = 100000
	   test<100000, 1, 1, double, 2, 10, 1>( output );
	   test<100000, 1, 1, double, 2, 20, 1>( output );
	   test<100000, 1, 1, double, 2, 50, 1>( output );
	   test<100000, 1, 1, double, 2, 100, 1>( output );
	   test<100000, 1, 1, double, 2, 500, 1>( output );
	   test<100000, 1, 1, double, 2, 1000, 1>( output );
		//n_pts = 500000
	   test<500000, 1, 1, double, 2, 10, 1>( output );
	   test<500000, 1, 1, double, 2, 20, 1>( output );
	   test<500000, 1, 1, double, 2, 50, 1>( output );
	   test<500000, 1, 1, double, 2, 100, 1>( output );
	   test<500000, 1, 1, double, 2, 500, 1>( output );
	}
	//dim = 2
	{
		//n_pts = 1000
	   test<1000, 2, 1, double, 2, 10,1>( output );
	   test<1000, 2, 1, double, 2, 20,1>( output);
	   test<1000, 2, 1, double, 2, 50,1>( output);
	   test<1000, 2, 1, double, 2, 100,1>( output);
		//n_pts = 5000
	   test<5000, 2, 1, double, 2, 10,1>( output);
	   test<5000, 2, 1, double, 2, 20,1>( output);
	   test<5000, 2, 1, double, 2, 50,1>( output);
	   test<5000, 2, 1, double, 2, 100,1>( output);
		//n_pts = 10000
	   test<10000, 2, 1, double, 2, 10,1>( output);
	   test<10000, 2, 1, double, 2, 20,1>( output);
	   test<10000, 2, 1, double, 2, 50,1>( output);
	   test<10000, 2, 1, double, 2, 100,1>( output);
		//n_pts = 50000
	   test<50000, 2, 1, double, 2, 10,1>( output);
	   test<50000, 2, 1, double, 2, 20,1>( output);
	   test<50000, 2, 1, double, 2, 50,1>( output);
	   test<50000, 2, 1, double, 2, 100,1>( output);
	   test<50000, 2, 1, double, 2, 500,1>( output);
		//n_pts = 100000
	   test<100000, 2, 1, double, 2, 10,1>( output);
	   test<100000, 2, 1, double, 2, 20,1>( output);
	   test<100000, 2, 1, double, 2, 50,1>( output);
	   test<100000, 2, 1, double, 2, 100,1>( output);
	   test<100000, 2, 1, double, 2, 500,1>( output);
	   test<100000, 2, 1, double, 2, 1000,1>( output);
		//n_pts = 500000
	   test<500000, 2, 1, double, 2, 10,1>( output);
	   test<500000, 2, 1, double, 2, 20,1>( output);
	   test<500000, 2, 1, double, 2, 50,1>( output);
	   test<500000, 2, 1, double, 2, 100,1>( output);
	   test<500000, 2, 1, double, 2, 500,1>( output);
	}
	//dim = 3
	{
		//n_pts = 1000
	   test<1000,3, 1, double, 2, 10,1>( output);
	   test<1000,3, 1, double, 2, 20,1>( output);
	   test<1000,3, 1, double, 2, 50,1>( output);
	   test<1000,3, 1, double, 2, 100,1>( output);
		//n_pts = 5000
	   test<5000,3, 1, double, 2, 10,1>( output);
	   test<5000,3, 1, double, 2, 20,1>( output);
	   test<5000,3, 1, double, 2, 50,1>( output);
	   test<5000, 3, 1, double, 2, 100,1>( output);
		//n_pts = 10000
	   test<10000,3, 1, double, 2, 10,1>( output);
	   test<10000,3, 1, double, 2, 20,1>( output);
	   test<10000,3, 1, double, 2, 50,1>( output);
	   test<10000,3, 1, double, 2, 100,1>( output);
		//n_pts = 50000
	   test<50000,3, 1, double, 2, 10,1>( output);
	   test<50000,3, 1, double, 2, 20,1>( output);
	   test<50000,3, 1, double, 2, 50,1>( output);
	   test<50000,3, 1, double, 2, 100,1>( output);
	   test<50000,3, 1, double, 2, 500,1>( output);
		//n_pts = 100000
	   test<100000,3, 1, double, 2, 10,1>( output);
	   test<100000,3, 1, double, 2, 20,1>( output);
	   test<100000,3, 1, double, 2, 50,1>( output);
	   test<100000,3, 1, double, 2, 100,1>( output);
	   test<100000,3, 1, double, 2, 500,1>( output);
	   test<100000,3, 1, double, 2, 1000,1>( output);
		//n_pts = 500000
	   test<500000,3, 1, double, 2, 10,1>( output);
	   test<500000,3, 1, double, 2, 20,1>( output);
	   test<500000,3, 1, double, 2, 50,1>( output);
	   test<500000,3, 1, double, 2, 100,1>( output);
	}

	return 0;
}



int main() {
	std::cout << "OPTICS Benchmark" << std::endl;

	//test<10000, 20, 100 * 100, double, 2, 1>( 10 );
	
	/*std::cout << "--- 2 dim ---" << std::endl;
	test<100000, 2, 100 * 100, double, 10, 1>( 10 );
	std::cout << std::endl << "--- 3 dim ---" << std::endl;
	test<100000, 3, 100 * 100, double, 10, 1>( 10 );
	std::cout << std::endl << "--- 4 dim ---" << std::endl;
	test<100000, 4, 100 * 100, double, 5, 1>( 10 );
	std::cout << std::endl << "--- 6 dim ---" << std::endl;
	test<100000, 6, 100 * 100, double, 5, 1>( 10 );
	std::cout << std::endl << "--- 10 dim ---" << std::endl;
	test<10000, 10, 100 * 100, double, 5, 1>( 10 );
	std::cout << std::endl << "--- 15 dim ---" << std::endl;
	test<10000, 15, 100 * 100, double, 5, 1>( 10 );
	std::cout << std::endl << "--- 20 dim ---" << std::endl;
	test<10000, 20, 100 * 100, double, 5, 1>( 10 );
	std::cout << std::endl << "--- 64 dim ---" << std::endl;
	test<1000, 64, 100 * 100, double, 5, 1>( 10 );
	std::cout << std::endl << "--- 128 dim ---" << std::endl;
	test<1000, 128, 100 * 100, double, 5, 1>( 10 );
	std::cout << std::endl << "--- 512 dim ---" << std::endl;
	test<1000, 512, 100 * 100, double, 5, 1>( 10 );
	*/

	test_all();


	return 0;

}