#ifndef soctype_h__
#define soctype_h__
/* Copyright (c) 2020 Matthew Madison */
typedef enum {
	TEGRA_SOCTYPE_BASE__,
	TEGRA_SOCTYPE_186,
	TEGRA_SOCTYPE_194,
	TEGRA_SOCTYPE_210,
	TEGRA_SOCTYPE_COUNT__
} tegra_soctype_t;
#define TEGRA_SOCTYPE_COUNT ((int) (TEGRA_SOCTYPE_COUNT__-(TEGRA_SOCTYPE_BASE__ + 1)))
#define TEGRA_SOCTYPE_INVALID ((tegra_soctype_t)(-1))

tegra_soctype_t soctype_get(void);
tegra_soctype_t soctype_from_chipid(unsigned int chipid);
const char *soctype_name(tegra_soctype_t soctype);

#endif /* soctype_h__ */
