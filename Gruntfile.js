'use strict';

module.exports = function (grunt) {
	grunt.initConfig({
		pkg: {
			name: "box2d"
		},
		clean: {
			src: ['dist']
		},
		concat: {
			options: {
				banner: '(function(){\n',
				footer: "\n}());",
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
				banner: '<%= banner %>'
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
			concat: {
				files: '<%= concat.dist.src %>',
				tasks: ['concat']
			},
			src: {
				files: '<%= jshint.src.src %>',
				tasks: ['default']
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
