import tkinter as tk
from tkinter import ttk

class WidgetFactory:
    def __init__(self,root):
        self.root = root
    
    def create_widget(self, widget_type, options):
        if widget_type == 'entry':
            entry_options = ['variable','width']
            options = {key: value for key, value in options.items() if key in entry_options}

            options['width']=10
            return tk.Entry(self.root,  **options)
        elif widget_type == 'button':
            return tk.Button(self.root, **options)
        elif widget_type == 'label':
            return tk.Label(self.root, **options)
        elif widget_type == 'checkbutton':
            check_options = ['variable']
            options = {key: value for key, value in options.items() if key in check_options}
            print(options)
            return tk.Checkbutton(self.root, **options)
        elif widget_type == 'scale':

            scale_options = ['from', 'to', 'orient', 'length', 'tickinterval', 'resolution','variable']
            options = {key: value for key, value in options.items() if key in scale_options}
            return tk.Scale(self.root, **options)
        elif widget_type == 'spinbox':
            return tk.Spinbox(self.root, **options)
        elif widget_type == 'combobox':
            options['width']=10
            options['textvariable'] = options.get('variable',None)
            combo_options = ['values','textvariable','width']
            options = {key: value for key, value in options.items() if key in combo_options}
            return ttk.Combobox(self.root, **options)
        else:
            raise ValueError(f"Unknown widget type: {widget_type}")


if __name__ == "__main__":
    root = tk.Tk()
    factory = WidgetFactory(root)
    
    # Example usage
    entry_widget = factory.create_widget('entry', {'bg': 'lightblue'})
    entry_widget.pack(pady=10)
    button_widget = factory.create_widget('button', {'text': 'Click Me', 'command': lambda: print("Button clicked!")})
    button_widget.pack(pady=10)
    label_widget = factory.create_widget('label', {'text': 'Hello, World!', 'font': ('Arial', 14)})
    label_widget.pack(pady=10)
    var = tk.BooleanVar(value = False)
    checkbutton_widget = factory.create_widget('checkbutton', {'text': 'Check me','variable':var})
    checkbutton_widget.pack(pady=10)

    scale_widget = factory.create_widget('scale', {'from': 0, 'to': 100, 'orient': 'horizontal'})
    scale_widget.pack(pady=10)

    spinbox_widget = factory.create_widget('spinbox', {'from': 0, 'to': 10, 'width': 5})
    spinbox_widget.pack(pady=10)

    combobox_widget = factory.create_widget('combobox',{'default_value':0,'values':[0,1]})
    combobox_widget.pack(pady=10)

    root.mainloop()