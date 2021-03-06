/*
** 
** Read <size,value> pairs from a file, like those generated by
** the pinpong_benchmark and compute the inflection
** points. I.e., places where the slope changes by more than some
** threshold.
** Rolf Riesen, September 2011, IBM Research, Ireland
*/
#include <stdio.h>
#include <stdlib.h>		/* For strtod() */
#include <unistd.h>		/* For getopt() */
#include <string.h>		/* For strerror() */
#include <errno.h>		/* For errno */
#include <math.h>		/* For fabs() */


#define FALSE				(0)
#define TRUE				(1)
#define DEFAULT_CHANGE_THRESHOLD	(0.1)
#define DEFAULT_SUPRESS_THRESHOLD	(0)
#define MAX_LINE			(1024)



int
main(int argc, char *argv[])
{

int ch, error;

char *pos;
int rc;
char line[MAX_LINE];
char *fname;
FILE *fp;
double threshold;

double value, size, slope;
double dummy;
double value_diff, size_diff;
double previous_size;
double previous_value;
double previous_slope;
double last_value;
double last_size;
double p;
int supress;
int user_supress;

int cnt;
int verbose;


    /* Defaults */
    error= FALSE;
    verbose= 0;
    fname= NULL;
    user_supress= DEFAULT_SUPRESS_THRESHOLD;
    threshold= DEFAULT_CHANGE_THRESHOLD;


    /* check command line args */
    while ((ch= getopt(argc, argv, "vf:s:t:")) != EOF)   {
	switch (ch)   {
	    case 'v':
		verbose++;
		break;
	    case 'f':
		fname= optarg;
		break;
	    case 's':
		user_supress= strtol(optarg, (char **)NULL, 0);
		break;
	    case 't':
		threshold= strtod(optarg, (char **)NULL);
		break;
	    default:
		error= TRUE;
		break;
	}
    }

    if (fname == NULL)   {
	error= TRUE;
	fprintf(stderr, "Must specify an input file using -f option!\n");
    }

    if (error)   {
	fprintf(stderr, "Usage: %s [-v {-v}] -f fname [-s supress] [-t threshold]\n", argv[0]);
	fprintf(stderr, "    -v              Increase verbosity\n");
	fprintf(stderr, "    -f fname        Input file name\n");
	fprintf(stderr, "    -s supress      Supress an inflection point unless it occurs more "
		"often than -s (Default %d)\n", DEFAULT_SUPRESS_THRESHOLD);
	fprintf(stderr, "    -t threshold    Percent of change to recognize an inflection point "
		"(Default %f)\n", DEFAULT_CHANGE_THRESHOLD);
	exit(-1);
    }

    /* Open the input file */
    fp= fopen(fname, "r");
    if (fp == NULL)   {
	fprintf(stderr, "Could not open the input file \"%s\": %s\n",
	    fname, strerror(errno));
	exit(-1);
    }



    cnt= 0;
    previous_size= 0.0;
    previous_value= 0.0;
    previous_slope= 0.0;
    supress= 10;


    /* Process the input file */
    while (fgets(line, MAX_LINE, fp) != NULL)   {
	/* Get rid of comments */
	pos= strchr(line, '#');
	if (pos)   {
	    *pos= '\0';
	}

	/* Now scan it */
	rc= sscanf(line, "%lf %lf %lf %lf %lf %lf %lf # %lf",
		&size, &dummy, &value, &dummy, &dummy, &dummy, &dummy, &dummy);

	if (rc < 3)   {
	    continue;
	}

	if (verbose > 1)   {
	    printf("Debug: Found size %f, value %f\n", size, value);
	}

	size_diff= size - previous_size;
	value_diff= value - previous_value;
	slope= fabs(value_diff / size_diff);

	if (cnt == 0)   {
	    /* Start the list */
	    printf("NetNICparams = %12.0f %12.0f\n", size, value * 1000.0);
	}

	previous_slope= (value - last_value) / (size - last_size);
	p= fabs(slope * 100.0 / previous_slope - 100.0);
	if (p > threshold)   {
	    /* We've reached an inflection point */
	    if (cnt > 1)   {
		if (supress > user_supress)   {
		    printf("NetNICparams = %12.0f %12.0f\n", previous_size, previous_value * 1000.0);
		    last_size= size;
		    last_value= value;
		    supress= 0;
		} else   {
		    supress++;
		}
	    }
	}

	previous_size= size;
	previous_value= value;

	cnt++;
    }

    printf("NetNICparams = %12.0f %12.0f\n", size, value * 1000.0);

    return 0;

}  /* end of main() */
