import customtkinter as ctk

class CollapsiblePane(ctk.CTkFrame):
    def __init__(self, parent: ctk.CTk | ctk.CTkFrame, contents:ctk.CTkFrame = None, expanded_text: str = "Collapse", collapsed_text: str = "Expand", default_state: bool = True):

        ctk.CTkFrame.__init__(self, parent)
        
        self.parent = parent
        self._contents = contents
        self._expanded_text = expanded_text
        self._collapsed_text = collapsed_text
        
        self._is_expanded = not default_state
        
        self.grid_columnconfigure(0, weight=1)

        self._button = ctk.CTkButton(self, text=self._collapsed_text, command=self._activate)
        self._button.grid(row=0, column=0, sticky="w")


        self.configure(fg_color="transparent", border_width=0)

        self._activate()
        
    def _activate(self):
        if(self._is_expanded):
            self._is_expanded = False
            self._button.configure(text=self._collapsed_text)
            if self._contents is not None:
                for cnt in self._contents:
                    cnt.grid_forget()
        else:
            self._is_expanded = True
            self._button.configure(text=self._expanded_text)
            if self._contents is not None:
                i = 1
                for cnt in self._contents:
                    py1 = 0
                    py2 = 20
                    if i == 1:
                        py1 = 20
                    if i == len(self._contents):
                        py2 = 0
                        
                    cnt.grid(row=i, column=0, padx=0, pady=(py1, py2), sticky="nsew")
                    i += 1
                
    def updateContent(self, new_content:list[ctk.CTkFrame]):
        if self._contents is not None:
            for cnt in self._contents:
                cnt.grid_forget()
                
        self._contents = new_content
        
        self._activate()

    