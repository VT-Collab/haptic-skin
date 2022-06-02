from tkinter import *
#
# window = tk.Tk()
# window.mainloop()



root = Tk()
root.title("Uncertainity Output")
update_time = 0.02
font = "Palatino Linotype"

# X_Y Uncertainty
myLabel1 = Label(root, text = "X-Y", font=(font, 40))
myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
textbox1 = Entry(root, width = 5, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
textbox1.grid(row = 0, column = 1,  pady = 10, padx = 20)
textbox1.insert(0,0)

# Z Uncertainty
myLabel2 = Label(root, text = "Z", font=("Palatino Linotype", 40))
myLabel2.grid(row = 1, column = 0, pady = 50, padx = 50)
textbox2 = Entry(root, width = 5, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
textbox2.grid(row = 1, column = 1,  pady = 10, padx = 20)
textbox2.insert(0,0)

# ROT Uncertainty
myLabel3 = Label(root, text = "ROT", font=("Palatino Linotype", 40))
myLabel3.grid(row = 2, column = 0, pady = 50, padx = 50)
textbox3 = Entry(root, width = 5, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
textbox3.grid(row = 2, column = 1,  pady = 10, padx = 20)
textbox3.insert(0,0)


root.mainloop()
