from django.contrib import admin

from .models import posts

class postsAdmin(admin.ModelAdmin):
	list_display = ["__unicode__", 'timestamp']
	fields = ['timestamp', 'author', "title", 'bodytext']
	class Meta:
		model = posts

admin.site.register(posts, postsAdmin)