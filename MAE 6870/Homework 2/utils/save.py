##### IMPORTS

##### SELF-DEFINED IMPORTS
import config

##### FIGURE SAVE FUNCTION (inputs list of all figures and names and saves to directory)
def SaveFigures(figs: list, fig_names: list) -> None:
    # saving all figures with names
    for fig, name in zip(figs, fig_names):
        fig.savefig(str(config.FIG_DIR / name), dpi=300, bbox_inches='tight')