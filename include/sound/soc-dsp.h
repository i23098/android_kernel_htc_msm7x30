/*
 * linux/sound/soc-dsp.h -- ALSA SoC DSP
 *
 * Author:		Liam Girdwood <lrg@slimlogic.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_SOC_DSP_H
#define __LINUX_SND_SOC_DSP_H

#include <sound/pcm.h>

struct snd_soc_dapm_widget;

/*
 * DSP trigger ordering. Triggering flexibility is required as some DSPs
 * require triggering before/after their clients/hosts.
 *
 * i.e. some clients may want to manually order this call in their PCM
 * trigger() whilst others will just use the regular core ordering.
 */
enum snd_soc_dsp_trigger {
	SND_SOC_DSP_TRIGGER_PRE		= 0,
	SND_SOC_DSP_TRIGGER_POST,
	SND_SOC_DSP_TRIGGER_BESPOKE,
};

/*
 * The DSP Backend state.
 */
enum snd_soc_dsp_link_state {
	SND_SOC_DSP_LINK_STATE_NEW	= 0,	/* newly created path */
	SND_SOC_DSP_LINK_STATE_FREE,			/* path to be dismantled */
	SND_SOC_DSP_LINK_STATE_HW_PARAMS,		/* path hw_params configured */
	SND_SOC_DSP_LINK_STATE_PREPARE,		/* path is prepared */
	SND_SOC_DSP_LINK_STATE_START,		/* path is started */
	SND_SOC_DSP_LINK_STATE_PAUSED,		/* path is paused */
};

struct snd_soc_dsp_params {
	struct snd_soc_pcm_runtime *be;
	struct snd_soc_pcm_runtime *fe;
	enum snd_soc_dsp_link_state state;
	struct list_head list_be;
	struct list_head list_fe;
	struct snd_pcm_hw_params params;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_state;
#endif
};

struct snd_soc_dsp_link {
	bool capture;
	bool playback;
	enum snd_soc_dsp_trigger trigger[2];
};

/* Runtime update - open/close Backend DSP paths depending on mixer updates */
int soc_dsp_runtime_update(struct snd_soc_dapm_widget *widget);

/* Backend DSP suspend and resume */
int soc_dsp_be_digital_mute(struct snd_soc_pcm_runtime *fe, int mute);
int soc_dsp_be_cpu_dai_suspend(struct snd_soc_pcm_runtime *fe);
int soc_dsp_be_ac97_cpu_dai_suspend(struct snd_soc_pcm_runtime *fe);
int soc_dsp_be_platform_suspend(struct snd_soc_pcm_runtime *fe);
int soc_dsp_be_cpu_dai_resume(struct snd_soc_pcm_runtime *fe);
int soc_dsp_be_ac97_cpu_dai_resume(struct snd_soc_pcm_runtime *fe);
int soc_dsp_be_platform_resume(struct snd_soc_pcm_runtime *fe);
#endif
