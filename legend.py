import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

def create_legend(output_png):
    """Создаёт цветовую легенду для отчётов"""
    fig, ax = plt.subplots(figsize=(6, 1))
    fig.subplots_adjust(bottom=0.5)
    
    cmap = plt.cm.viridis
    norm = plt.Normalize(vmin=0, vmax=100)
    
    cb = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap),
                     cax=ax, orientation='horizontal',
                     label='Отражаемость (%)')
    
    fig.savefig(output_png, dpi=300, bbox_inches='tight')

create_legend("reflectivity_legend.png")