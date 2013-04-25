'use strict';

module.exports = function (grunt) {
	grunt.initConfig({
		pkg: {
			name: "Box2D"
		},
		clean: {
			src: ['dist']
		},
		concat: {
			options: {
				banner: '<%= banner %>',
				stripBanners: true
			},
			dist: {
				src: [
					"src/start.js",
					"src/Common/*.js",
					"src/Common/Math/*.js",
					"src/Collision/*.js",
					"src/Collision/Shapes/*.js",
					"src/Dynamics/*.js",
					"src/Dynamics/Contacts/*.js",
					"src/Dynamics/Controllers/*.js",
					"src/Dynamics/Joints/*.js",
					"src/end.js"
				],
				dest: 'dist/<%= pkg.name %>.js'
			}
		},
		uglify: {
			options: {
				banner: '<%= banner %>',
				beautify: true
			},
			dist: {
				src: '<%= concat.dist.dest %>',
				dest: 'dist/<%= pkg.name %>.min.js'
			}
		},
		jshint: {
			src: {
				options: {
					jshintrc: '.jshintrc'
				},
				src: ['src/**/*.js']
			}
		},
		watch: {
			src: {
				files: '<%= jshint.src.src %>',
				tasks: ['jshint:src', 'qunit']
			}
		}
	});

	// These plugins provide necessary tasks.
	grunt.loadNpmTasks('grunt-contrib-clean');
	grunt.loadNpmTasks('grunt-contrib-concat');
	grunt.loadNpmTasks('grunt-contrib-uglify');
	grunt.loadNpmTasks('grunt-contrib-jshint');
	grunt.loadNpmTasks('grunt-contrib-watch');

	// Default task.
	grunt.registerTask('default', ['jshint', 'clean', 'concat', 'uglify']);

};
