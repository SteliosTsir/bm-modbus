/* Tiny fork-per-test harness: a crash or sanitizer abort in one test is reported, not fatal. */
#ifndef MBTEST_H
#define MBTEST_H
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/wait.h>

struct tcase { const char *name; void (*fn)(void); struct tcase *next; };
static struct tcase *tests_head, **tests_tail = &tests_head;

#define TEST(n) static void n(void); \
	__attribute__((constructor)) static void reg_##n(void) { \
		static struct tcase t = { #n, n, 0 }; *tests_tail = &t; tests_tail = &t.next; } \
	static void n(void)

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "CHECK failed %s:%d: %s\n", \
	__FILE__, __LINE__, #c); _exit(1); } } while (0)

#define CHECK_BYTES(got, want, n) do { \
	const uint8_t *g_ = (got), *w_ = (want); size_t n_ = (n); \
	if (memcmp(g_, w_, n_)) { fprintf(stderr, "CHECK_BYTES failed %s:%d\n  got:  ", __FILE__, __LINE__); \
		for (size_t i_ = 0; i_ < n_; i_++) fprintf(stderr, "%02X ", g_[i_]); \
		fprintf(stderr, "\n  want: "); \
		for (size_t i_ = 0; i_ < n_; i_++) fprintf(stderr, "%02X ", w_[i_]); \
		fprintf(stderr, "\n"); _exit(1); } } while (0)

static int mbtest_main(int argc, char **argv)
{
	int pass = 0, fail = 0, verbose = 0;
	const char *filter = NULL;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) verbose = 1; else filter = argv[i];
	}
	for (struct tcase *t = tests_head; t; t = t->next) {
		if (filter && !strstr(t->name, filter)) continue;
		char path[] = "/tmp/mbtestXXXXXX";
		int fd = mkstemp(path);
		fflush(stdout);
		pid_t p = fork();
		if (p == 0) {
			dup2(fd, 2);
			t->fn();
			_exit(0);
		}
		int st = 0;
		waitpid(p, &st, 0);
		close(fd);
		int ok = WIFEXITED(st) && WEXITSTATUS(st) == 0;
		if (ok) pass++; else fail++;
		printf("%s %s\n", ok ? "PASS" : "FAIL", t->name);
		if (!ok || verbose) {
			FILE *f = fopen(path, "r"); char line[512]; int n = 0;
			while (f && fgets(line, sizeof line, f) && n < (verbose ? 40 : 4)) {
				if (!ok && !strstr(line, "CHECK") && !strstr(line, "ERROR") &&
				    !strstr(line, "runtime error") && !strstr(line, "got:") &&
				    !strstr(line, "want:") && !strstr(line, " #1 ") && !strstr(line, " #2 ")) continue;
				printf("     %s", line); n++;
			}
			if (f) fclose(f);
			if (!ok && WIFSIGNALED(st)) printf("     (killed by signal %d)\n", WTERMSIG(st));
		}
		unlink(path);
	}
	printf("\n%d passed, %d failed\n", pass, fail);
	return fail ? 1 : 0;
}
#endif
